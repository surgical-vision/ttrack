#include<boost/filesystem.hpp>

#include "../../../include/ttrack/track/pwp3d/mono_pwp3d.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

MonoPWP3D::MonoPWP3D(boost::shared_ptr<MonocularCamera> camera) : camera_(camera), PWP3D(camera->Width(), camera->Height()) {

  LoadShaders();

}


void MonoPWP3D::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  if (curr_step == NUM_STEPS) {
    curr_step = 0;
  }

  ++curr_step;

  frame_ = frame;

  std::vector<float> jacs(7, 0);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();

  cv::Mat sdf_image, front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera_, sdf_image, front_intersection_image, back_intersection_image);

  //setup point registration for first frame
  if (!point_registration_){
    point_registration_.reset(new PointRegistration(camera_));
    point_registration_->ComputeDescriptorsForPointTracking(frame_->GetImageROI(), front_intersection_image, current_model->GetBasePose());
  }

  float fg_area, bg_area = 0;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;

  size_t error = 0;

  cv::Mat classification_image = frame_->GetClassificationMap();

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //-log(H * P_f + (1-H) * P_b)
        error += GetErrorValue(classification_image, r, c, sdf_im_data[i], 1.0f, fg_area, bg_area);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], fg_area, bg_area);

        int shifted_i = i;

        //find the closest point on the contour if this point is outside the contour
        if (sdf_im_data[i] < 0.0) {
          int closest_r, closest_c;
          bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
          if (!found) continue; //should this be allowed to happen?
          shifted_i = closest_r * sdf_image.cols + closest_c;
        }

        cv::Matx<float, 1, 7> jacs;
        for (int j = 0; j < 7; ++j){
          jacs(j) = 0.0f;
        }

        const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
        const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

        //update the jacobian
        
        UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera_->Fx(), camera_->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        
        jacobian += jacs.t();
        hessian_approx += (jacs.t() * jacs);

      }
    }
  }

}

