#include <boost/filesystem.hpp>
#include <cinder/app/App.h>

#include "../../../include/ttrack/track/localizer/levelsets/stereo_pwp3d.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

#ifdef USE_CUDA
#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"
#endif 

using namespace ttrk;

#define GRAD_DESCENT

StereoPWP3D::StereoPWP3D(boost::shared_ptr<StereoCamera> camera) : PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()), stereo_camera_(camera) {

  LoadShaders();

}

float StereoPWP3D::DoAlignmentStep(boost::shared_ptr<Model> current_model){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> region_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> region_hessian_approx = cv::Matx<float, 7, 7>::zeros();

  if (use_level_sets_){
    ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), region_jacobian, region_hessian_approx, error);
    //ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), region_jacobian, region_hessian_approx, error);
  }
  
  if (point_registration_)
    ComputePointRegistrationJacobian(current_model, region_jacobian, region_hessian_approx);


  float current_score = 0.0f; //actual score
  float best_score = 0.0f; //best achieveable score given the contour
  cv::Mat sdf_image;
  ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), sdf_image, cv::Mat(), cv::Mat());

  ComputeScores(sdf_image, stereo_frame->GetLeftClassificationMap(), current_score, best_score);

  if (best_score > 0)
    region_scores.push_back(current_score / best_score);

#ifdef GRAD_DESCENT

  std::vector<float> jacs = ScaleRigidJacobian(region_jacobian);
  current_model->UpdatePose(jacs);

#else

  std::vector<float> jacs(7);
  jacobian = hessian_approx.inv() * jacobian;
  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }
  current_model->UpdatePose(jacs);

#endif

  if (point_registration_)
    point_registration_->UpdatePointsOnModelAfterDerivatives(current_model, current_model->GetBasePose());

  return error;

}

float StereoPWP3D::DoPointBasedAlignmentStepForLeftEye(boost::shared_ptr<Model> current_model){

  if (!point_registration_) return 0.0;

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();

  ComputePointRegistrationJacobian(current_model, jacobian, hessian_approx);
  
#ifdef GRAD_DESCENT

  std::vector<float> jacs = ScaleRigidJacobian(jacobian);
  current_model->UpdatePose(jacs);

#else

  std::vector<float> jacs(7);
  jacobian = hessian_approx.inv() * jacobian;
  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }
  current_model->UpdatePose(jacs);	

#endif

  if (point_registration_)
    point_registration_->UpdatePointsOnModelAfterDerivatives(current_model, current_model->GetBasePose());

  return error;

}

float StereoPWP3D::DoRegionBasedAlignmentStepForRightEye(boost::shared_ptr<Model> current_model){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();
  ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), jacobian, hessian_approx, error);


#ifdef GRAD_DESCENT

  std::vector<float> jacs = ScaleRigidJacobian(jacobian);
  current_model->UpdatePose(jacs);

#else
  
  std::vector<float> jacs(7);
  jacobian = hessian_approx.inv() * jacobian;
  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }
  current_model->UpdatePose(jacs);

#endif

  return error;

}

float StereoPWP3D::DoRegionBasedAlignmentStepForLeftEye(boost::shared_ptr<Model> current_model){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();
  ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), jacobian, hessian_approx, error);

#ifdef GRAD_DESCENT

  std::vector<float> jacs = ScaleRigidJacobian(jacobian);
  current_model->UpdatePose(jacs);

#else

  jacobian = hessian_approx.inv() * jacobian;
  std::vector<float> jacs(7, 0);
  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }
  current_model->UpdatePose(jacs);

#endif



  return error;

}

void StereoPWP3D::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

#ifdef USE_CERES

  TrackTargetCeresOptimization(current_model, frame);

#else

  frame_ = frame;


  if (!current_model->cam) current_model->cam = stereo_camera_->left_eye();


  if (curr_step == 0) {

    cv::Mat left_sdf_image;
    cv::Mat front_intersection_image, back_intersection_image, front_normal_image;
    cv::Mat sdf_image(frame_->GetImage().rows, frame_->GetImage().cols, CV_32FC1), right_sdf_image;
    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, front_intersection_image, back_intersection_image);
    Localizer::ResetOcclusionImage();
    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, front_intersection_image, back_intersection_image);

    auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);


    if (current_model->NeedsModelRetrain()){

      cv::Mat label_image = cv::Mat::zeros(left_sdf_image.size(), CV_8UC1);
      for (int r = 0; r < label_image.rows; ++r){
        for (int c = 0; c < label_image.cols; ++c){
          if (left_sdf_image.at<float>(r, c) > 0){
            label_image.at<unsigned char>(r, c) = 1; 
          }
        }
      }

      current_model->RetrainModel(stereo_frame->GetLeftImage(), left_sdf_image, label_image);
    }

    
    left_sdf_image.copyTo(sdf_image(cv::Rect(0, 0, left_sdf_image.cols, left_sdf_image.rows)));
    right_sdf_image.copyTo(sdf_image(cv::Rect(left_sdf_image.cols, 0, left_sdf_image.cols, left_sdf_image.rows)));
    current_model->ClassifyFrame(frame_, sdf_image);

    if (point_registration_ && !current_model->mps.is_initialised){
      point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);
      point_registration_->InitializeTracker(stereo_frame->GetLeftImage(), current_model);
    }
    else if (point_registration_ && current_model->mps.is_initialised){
      point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);
      point_registration_->TrackLocalPoints(stereo_frame->GetLeftImage(), current_model);
    }

  }

  
  //float left_error = DoRegionBasedAlignmentStepForLeftEye(current_model);
  // float right_error = DoRegionBasedAlignmentStepForRightEye(current_model);
  //float point_error = DoPointBasedAlignmentStepForLeftEye(current_model);

  float error = DoAlignmentStep(current_model);

  //UpdateWithErrorValue(left_error + right_error + point_error);
  //errors_.push_back(left_error + right_error + point_error);
  UpdateWithErrorValue(error);
  errors_.push_back(error);

#endif

}

void StereoPWP3D::ComputeJacobiansForEyeCUDA(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){

}

void StereoPWP3D::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){

  cv::Mat sdf_image, front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, sdf_image, front_intersection_image, back_intersection_image);

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float fg_area = 1.0f, bg_area = 1.0f;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);
  
  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;


  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        if (occlusion_image.at<float>(r, c) < (front_intersection_data[(i * 3) + 2] - 0.1)) {
          continue;
        }

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
        if (camera == stereo_camera_->left_eye())
          UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else if (camera == stereo_camera_->right_eye())
          UpdateJacobianRightEye(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else{
          ci::app::console() << "Unsupported camera!" << std::endl;
          throw std::runtime_error("");
        }

        jacobian += jacs.t();
        hessian_approx += (jacs.t() * jacs);

      }
    }
  }
}

void StereoPWP3D::UpdateJacobianRightEye(const float region_agreement, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, 7> &jacobian){

  //
  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  ////compute the derivatives w.r.t. left camera pose (this is because we move model w.r.t. left eye)
  ci::Vec3f front_intersection_left_eye = stereo_camera_->TransformPointFromRightToLeft(front_intersection_point);
  ci::Vec3f back_intersection_left_eye = stereo_camera_->TransformPointFromRightToLeft(front_intersection_point);
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_left_eye, 0);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_left_eye, 0);

  //use the 'inverse' of the point transform. as we store the relative orientation (which is already the inverse of the point transform) just use that here.
  const ci::Matrix33f inverse_rotation = stereo_camera_->ciExtrinsicRotation();

  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < model->GetBasePose().GetNumDofs(); ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];

    if (sdf == 0.0f){

      const float deriv_x_front =
        front_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_x = dsdf_dx * (fx * z_inv_sq_front * deriv_x_front);

      const float deriv_y_front =
        front_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_y = dsdf_dy * (fy * z_inv_sq_front * deriv_y_front);

      //pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      const float pval = DeltaFunction(sdf) * (deriv_x + deriv_y);

      jacobian(dof) = region_agreement * pval;
    }

    else{

      const float deriv_x_front =
        front_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_x_back =
        back_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_back[2]))
        -
        back_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_back[2]));

      const float deriv_x = dsdf_dx * ((fx * z_inv_sq_front * deriv_x_front) + (fx * z_inv_sq_back * deriv_x_back));

      const float deriv_y_front =
        front_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_y_back =
        back_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_back[2]))
        -
        back_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_back[2]));
      const float deriv_y = dsdf_dy * ((fy * z_inv_sq_front * deriv_y_front) + (fy * z_inv_sq_back * deriv_y_back));

      //pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      const float pval = DeltaFunction(sdf) * (deriv_x + deriv_y);

      jacobian(dof) = region_agreement * pval;

    }
  }

}

#ifdef USE_CERES

struct CeresCostFunction : public ceres::SizedCostFunction<401860, 3, 4> {

public:

  CeresCostFunction(StereoPWP3D &pwp3d) : pwp3d_(pwp3d) {}
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

    return pwp3d_(parameters, residuals, jacobians);

  }

  StereoPWP3D &pwp3d_;


};

void StereoPWP3D::DoEyeCeres(double const *const *parameters, double *residuals, double **jacobians, bool IS_LEFT) const {

  ci::app::console() << "start" << std::endl;

  const ci::Vec3f translation(parameters[0][0], parameters[0][1], parameters[0][2]);
  ci::Quatf rotation(parameters[1][0], parameters[1][1], parameters[1][2], parameters[1][3]);

  Pose p(rotation, translation);
  current_model_->SetBasePose(p);

  cv::Mat left_image = frame_->GetImageROI();
  cv::Mat classification_image = frame_->GetClassificationMapROI();

  cv::Mat sdf_image, front_intersection_image, back_intersection_image;
  const_cast<StereoPWP3D *>(this)->ProcessSDFAndIntersectionImage(current_model_, stereo_camera_->left_eye(), sdf_image, front_intersection_image, back_intersection_image);

  float fg_area = 1.0f, bg_area = 1.0f;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  float residual_sum = 0;

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;

  ci::app::console() << "inside loop" << std::endl;

  size_t ceres_matrix_index = 0;

  for (int row = 5; row < left_image.rows - 5; ++row){

    for (int col = 5; col < left_image.cols - 5; ++col, ++ceres_matrix_index){
      
      const int index = (left_image.cols*row) + col;

      const float sdf_value = sdf_im_data[index];

      cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(row, col);
      float pixel_probability = re[1];
      for (int i = 2; i < 5; ++i){
        pixel_probability = std::max(pixel_probability, re[i]);
      }
      float Pf = pixel_probability / fg_area;
      float Pb = re[0] / bg_area;
      //if (SAFE_EQUALS<float>(Pf, 0) && SAFE_EQUALS<float>(Pb, 1)){
      //  Pf += 0.0005;
      //  Pb -= 0.0005;
      //}
      //else if (SAFE_EQUALS<float>(Pf, 1) && SAFE_EQUALS<float>(Pb, 0)){
      //  Pf -= 0.0005;
      //  Pb += 0.0005;
      //}
      if (SAFE_EQUALS<float>(Pf, 0)){
        Pf += 0.0000000000000001;
      }
      if (SAFE_EQUALS<float>(Pb, 0)){
        Pb += 0.0000000000000001;
      }

      if (sdf_value <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_value >= -float(HEAVYSIDE_WIDTH) + 1e-1){


        int i = index;
        int r = row;
        int c = col;
        int shifted_i = i;

        if (jacobians != NULL && (jacobians[0] != NULL || jacobians[1] != NULL)){

          //region agreement
          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], fg_area, bg_area);



          //find the closest point on the contour if this point is outside the contour
          if (sdf_im_data[i] < 0.0) {
            int closest_r, closest_c;
            bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
            if (!found) {

              residuals[ceres_matrix_index] = 0;

              if (jacobians != NULL){
                if (jacobians[0] != NULL){

                  for (int j_idx = 0; j_idx < 3; ++j_idx)
                    jacobians[0][ceres_matrix_index * 3 + j_idx] = 0;

                }
                if (jacobians[1] != NULL){

                  for (int j_idx = 0; j_idx < 4; ++j_idx)
                    jacobians[1][ceres_matrix_index * 4 + j_idx] = 0;

                }
              }

              continue; //should this be allowed to happen?
            }
            shifted_i = closest_r * sdf_image.cols + closest_c;
          }

          cv::Matx<float, 1, 7> jacs;
          for (int j = 0; j < 7; ++j){
            jacs(j) = 0.0f;
          }

          const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
          const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

          //update the jacobian
          if (IS_LEFT)
            const_cast<StereoPWP3D *>(this)->UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model_, jacs);
          else
            const_cast<StereoPWP3D *>(this)->UpdateJacobianRightEye(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->right_eye()->Fx(), stereo_camera_->right_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model_, jacs);


          if (jacobians != NULL){
            //ci::app::console() << "450" << std::endl;
            if (jacobians[0] != NULL){
              //ci::app::console() << "452";
              for (int j_idx = 0; j_idx < 3; ++j_idx)
                jacobians[0][ceres_matrix_index * 3 + j_idx] = jacs(j_idx);
              //ci::app::console() << "done" << std::endl;


            }
            if (jacobians[1] != NULL){
              //ci::app::console() << "460";
              for (int j_idx = 0; j_idx < 4; ++j_idx)
                jacobians[1][ceres_matrix_index * 4 + j_idx] = jacs(3 + j_idx);
              //ci::app::console() << "done" << std::endl;


            }
          }
        }

        //ci::app::console() << "470";
        residuals[ceres_matrix_index] = -log((double)(HeavisideFunction(sdf_value)*Pf + (1 - HeavisideFunction(sdf_value))*Pb) + 0.000001);
        //ci::app::console() << "done" << std::endl;
        residual_sum += residuals[ceres_matrix_index];

      }
      else{
        if (sdf_value > 0){
          //ci::app::console() << "478";
          residuals[ceres_matrix_index] = -log(Pf);
          //ci::app::console() << "done" << std::endl;
        }
        else{
          //ci::app::console() << "483";
          residuals[ceres_matrix_index] = -log(Pb);
          //ci::app::console() << "done" << std::endl;
        }

        if (jacobians != NULL){
          //ci::app::console() << "489";
          if (jacobians[0] != NULL){

            //ci::app::console() << "492";
            for (int j_idx = 0; j_idx < 3; ++j_idx)
              jacobians[0][ceres_matrix_index * 3 + j_idx] = 0;
            //ci::app::console() << "done" << std::endl;
          }
          if (jacobians[1] != NULL){

            //ci::app::console() << "499";
            for (int j_idx = 0; j_idx < 4; ++j_idx)
              jacobians[1][ceres_matrix_index * 4 + j_idx] = 0;
            //ci::app::console() << "done" << std::endl;
          }
        }

      }
    

    }
  }

  ci::app::console() << "new parameter test = " << translation << " and " << rotation << " gives residual = " << residual_sum << std::endl;

}


bool StereoPWP3D::operator() (double const *const *parameters, double *residuals, double **jacobians) const{

  
  DoEyeCeres(parameters, residuals, jacobians, true);

  return true;

}

bool StereoPWP3D::operator() (double const* const* parameters, double* residuals) const{

  const ci::Vec3f translation(100 * parameters[0][0], 100 * parameters[0][1], 100 * parameters[0][2]);
  ci::Quatf rotation(parameters[1][0], parameters[1][1], parameters[1][2], parameters[1][3]);

  Pose p(rotation, translation);
  current_model_->SetBasePose(p);

  cv::Mat left_image = frame_->GetImageROI();
  cv::Mat classification_image = frame_->GetClassificationMapROI();


  cv::Mat sdf_image, front_intersection_image, back_intersection_image;
  const_cast<StereoPWP3D *>(this)->ProcessSDFAndIntersectionImage(current_model_, stereo_camera_->left_eye(), sdf_image, front_intersection_image, back_intersection_image);

  float fg_area = 1.0f, bg_area = 1.0f;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  float residual_sum = 0;

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;

  for (int row = 5; row < left_image.rows - 5; ++row){
    for (int col = 5; col < left_image.cols - 5; ++col){
      const int index = (left_image.cols*row) + col;

      const float sdf_value = sdf_im_data[index];

      cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(row, col);
      float pixel_probability = re[1];
      for (int i = 2; i < 5; ++i){
        pixel_probability = std::max(pixel_probability, re[i]);
      }
      float Pf = pixel_probability / fg_area;
      float Pb = re[0] / bg_area;
      if (SAFE_EQUALS<float>(Pf, 0) && SAFE_EQUALS<float>(Pb, 1)){
        Pf += 0.05;
        Pb -= 0.05;
      }
      else if (SAFE_EQUALS<float>(Pf, 1) && SAFE_EQUALS<float>(Pb, 0)){
        Pf -= 0.05;
        Pb += 0.05;
      }

      if (sdf_value <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_value >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //region agreement


        residuals[index] = -log((double)(HeavisideFunction(sdf_value)*Pf + (1 - HeavisideFunction(sdf_value))*Pb));
        residual_sum += residuals[index];

      }
      else{
        if (sdf_value > 0){
          residuals[index] = -log(Pf);
        }
        else{
          residuals[index] = -log(Pb);

        }
      }
    }
  }

  ci::app::console() << "new parameter test = " << translation << " and " << rotation << " gives residual = " << residual_sum << std::endl;

  return true;


}

void StereoPWP3D::TrackTargetCeresOptimization(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  current_model_ = current_model;
  cv::Mat left_image = frame_->GetImageROI();

  std::vector<float> pose_parameters;
  current_model->GetPose(pose_parameters);

  double **parameters = new double*[2];
  //double **parameters = new double*[1];
  parameters[0] = new double[3];
  parameters[1] = new double[4];

  static bool init_run = true;
  if (init_run){
    parameters[0][0] = pose_parameters[0] + 0.225;
    parameters[0][1] = pose_parameters[1] + 0.825;
    parameters[0][2] = pose_parameters[2] + 0.125;
    init_run = false;
  }
  else{
    parameters[0][0] = pose_parameters[0];
    parameters[0][1] = pose_parameters[1];
    parameters[0][2] = pose_parameters[2];
  }
  parameters[1][0] = pose_parameters[3]; //w
  parameters[1][1] = pose_parameters[4]; //x
  parameters[1][2] = pose_parameters[5]; //y
  parameters[1][3] = pose_parameters[6]; //z


  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);
  ceres::Solver::Options options;

  ceres::CostFunction *cost_function = new CeresCostFunction(*this);
  //ceres::DynamicNumericDiffCostFunction<StereoPWP3D> *cost_function = new ceres::DynamicNumericDiffCostFunction<StereoPWP3D>(this, ceres::Ownership::DO_NOT_TAKE_OWNERSHIP, 0.0001);
  //cost_function->AddParameterBlock(3);
  //cost_function->AddParameterBlock(4);
  //cost_function->SetNumResiduals(left_image.rows*left_image.cols);

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(parameters[0]);
  parameter_blocks.push_back(parameters[1]);


  ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, NULL, parameter_blocks);

  ceres::QuaternionParameterization *qp = new ceres::QuaternionParameterization();
  
  problem.SetParameterization(parameters[1], qp);
  

  options.linear_solver_type = ceres::DENSE_QR;

  //minimizer can be line search or trust region
  //line search - choose size and then find direction that minimizes the function
  //trust region - choose a direction 
  options.minimizer_type = ceres::LINE_SEARCH; 
  options.line_search_direction_type = ceres::BFGS;
  //options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  ci::app::console() << "Done" << summary.FullReport() << "\n" << std::endl;

  const ci::Vec3f translation(parameters[0][0], parameters[0][1], parameters[0][2]);
  ci::Quatf rotation(parameters[1][0], parameters[1][1], parameters[1][2], parameters[1][3]);
  //ci::Quatf rotation = current_model_->GetBasePose().GetRotation();


  //ci::app::console() << "new parameter test = " << translation << " and " << rotation << std::endl;
  //rotation.normalize();

  current_model_->SetBasePose(Pose(rotation, translation));

  UpdateWithErrorValue(10);

  delete cost_function;
  delete qp;
  delete[] parameters[0];
  delete[] parameters[1];
  delete[] parameters;

}





#endif