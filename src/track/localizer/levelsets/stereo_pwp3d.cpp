#include <boost/filesystem.hpp>
#include <cinder/app/App.h>

#include "../../../include/ttrack/track/localizer/levelsets/stereo_pwp3d.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

#define GRAD_DESCENT

StereoPWP3D::StereoPWP3D(boost::shared_ptr<StereoCamera> camera) : PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()), stereo_camera_(camera) {

  LoadShaders();

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


  point_registration_->UpdatePointsOnModelAfterDerivatives(current_model->GetBasePose());

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

  frame_ = frame;

  if (curr_step == NUM_STEPS) {

    cv::Mat front_intersection_image, back_intersection_image, front_normal_image;
    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), front_intersection_image, back_intersection_image);

    curr_step = 0;

    auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

    if (!point_registration_){

      point_registration_.reset(new PointRegistration(stereo_camera_->left_eye()));
      point_registration_->SetFrontIntersectionImage(front_intersection_image);
      point_registration_->InitializeTracker(stereo_frame->GetLeftImage(), current_model->GetBasePose());
    
    }
    else{

      point_registration_->UpdatePose(current_model->GetBasePose());
      point_registration_->SetFrontIntersectionImage(front_intersection_image);
      point_registration_->TrackLocalPoints(stereo_frame->GetLeftImage());

    }

  }

  ++curr_step;
  
  float left_error = DoRegionBasedAlignmentStepForLeftEye(current_model);
  float right_error = DoRegionBasedAlignmentStepForRightEye(current_model);
  //float point_error = 0;
  float point_error = DoPointBasedAlignmentStepForLeftEye(current_model);

  UpdateWithErrorValue(left_error + right_error + point_error);
  errors_.push_back(left_error + right_error + point_error);

}

void StereoPWP3D::ComputeJacobiansForEyeCUDA(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){

}


void StereoPWP3D::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){

  cv::Mat sdf_image, front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, sdf_image, front_intersection_image, back_intersection_image);

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float fg_area = 1.0f, bg_area = 1.0f;
  size_t contour_area = 0;
  //ComputeAreas(sdf_image, fg_area, bg_area, contour_area);
  
  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;


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
        if (camera == stereo_camera_->left_eye())
          UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else if (camera == stereo_camera_->right_eye())
          UpdateJacobianRightEye(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else
          throw std::runtime_error("");

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
