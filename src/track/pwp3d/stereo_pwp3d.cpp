#include <boost/filesystem.hpp>

#include "../../../include/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"

using namespace ttrk;

void StereoPWP3D::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){
   
  frame_ = frame;

  NUM_STEPS = 1;

  ////using namespace ceres;
  //// Build the problem.
  ////ceres::Problem problem;
  //// The variable to solve for with its initial value.
  //double initial_x = 5.0;
  //double x = initial_x;

  //// Set up the only cost function (also known as residual). This uses
  //// auto-differentiation to obtain the derivative (jacobian).
  ////ceres::CostFunction* cost_function =
  ////  new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  ////problem.AddResidualBlock(cost_function, NULL, &x);

  //// Run the solver!
  ////Solver::Options options;
  ////options.linear_solver_type = ceres::DENSE_QR;
  ////options.minimizer_progress_to_stdout = true;
  ////Solver::Summary summary;
  ////Solve(options, &problem, &summary);

  cv::Mat left_sdf_image, left_front_intersection, left_back_intersection;
  cv::Mat right_sdf_image, right_front_intersection, right_back_intersection;
  const int NUM_JACS = current_model->GetBasePose().GetNumDofs();
  float *jacs = new float[NUM_JACS]; 
  

  //iterate until steps or convergences
  for (size_t step = 0; step < NUM_STEPS; ++step){

    memset(jacs, 0, NUM_JACS * sizeof(float));

    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, left_front_intersection, left_back_intersection);
    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, right_front_intersection, right_back_intersection);

    cv::Mat dsdf_dx, dsdf_dy;
    cv::Scharr(left_sdf_image, dsdf_dx, CV_32FC1, 1, 0);
    cv::Scharr(left_sdf_image, dsdf_dy, CV_32FC1, 0, 1);

    cv::Mat &classification_image = frame_->GetClassificationMapROI();
    cv::Mat &front_intersection_image = left_front_intersection;
    cv::Mat &back_intersection_image = right_front_intersection;
    cv::Mat &sdf_image = left_sdf_image;

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;
    float *dsdf_dx_data = (float *)dsdf_dx.data;
    float *dsdf_dy_data = (float *)dsdf_dy.data;
    
    for (int r = 5; r < classification_image.rows-5; ++r){
      for (int c = 5; c < classification_image.cols-5; ++c){
        
        int i = r*classification_image.cols + c;
          
        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(r, c, sdf_im_data[i]);

          //find the closest point on the contour if this point is outside the contour
          if (sdf_im_data[i] < 0.0) {
            int closest_r, closest_c;
            bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
            if (!found) continue; //should this be allowed to happen?
            i = closest_r * sdf_image.cols + closest_c;
          }

          //update the jacobian
          UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx_data[i], dsdf_dy_data[i], stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), cv::Vec3f(&front_intersection_data[i * 3]), cv::Vec3f(&back_intersection_data[i * 3]), current_model, jacs) ;

        }
      }
    }

    ci::Vec3f translation_jac(jacs[0], jacs[1], jacs[2]);
    translation_jac.normalize();

    ci::Vec4f rotation_jac(0, 0, 0, 0);// jacs[3], jacs[4], jacs[5], jacs[6]);
    //rotation_jac.normalize(); 

    current_model->UpdatePose(std::vector<float>({ -translation_jac[0], -translation_jac[1], -translation_jac[2],
      rotation_jac[0], rotation_jac[1], rotation_jac[2], rotation_jac[3], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }));

  }

  delete [] jacs;

}

/*
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};
*/


