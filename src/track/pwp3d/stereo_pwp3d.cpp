#include <boost/filesystem.hpp>
#include <cinder/app/App.h>

#include "../../../include/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"

using namespace ttrk;

void StereoPWP3D::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){
   
  if (curr_step == NUM_STEPS) {
    curr_step = 0;
  }

  ++curr_step;

  frame_ = frame;

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float left_error = 0.0f, right_error = 0.0f;
  //ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), jacobian, hessian_approx, left_error);
  ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), jacobian, hessian_approx, right_error);

  UpdateWithErrorValue(left_error + right_error);
  errors_.push_back(left_error + right_error);

  ci::app::console() << "Jacobian before  = [";
  for (size_t v = 0; v < 7; ++v){
    ci::app::console() << jacobian(v) << ",";
  }
  ci::app::console() << "]" << std::endl;


  jacobian = hessian_approx.inv() * jacobian;
  
  //float t_scale = std::max(std::abs(jacobian(0)), std::abs(jacobian(1)));
  //t_scale = std::max(t_scale, std::abs(jacobian(2)));
  //float r_scale = std::max(std::abs(jacobian(3)), std::abs(jacobian(4)));
  //r_scale = std::max(r_scale, std::abs(jacobian(5)));
  //r_scale = std::max(r_scale, std::abs(jacobian(6)));
  //jacobian(0) *= (0.03 / t_scale);
  //jacobian(1) *= (0.03 / t_scale);
  //jacobian(2) *= (0.05 / t_scale);
  //jacobian(3) *= (0.0013 / r_scale);
  //jacobian(4) *= (0.0013 / r_scale);
  //jacobian(5) *= (0.0013 / r_scale);
  //jacobian(6) *= (0.0013 / r_scale);
  
  //jacobian(0) *= 0;// (0.0025f);
  //jacobian(1) = -0;//(0.0025f);
  //jacobian(2) *= 0;//(0.0025f);
  //jacobian(3) *= 0;//(0.000006f);
  //jacobian(4) *= 0;//(0.000006f);
  //jacobian(5) *= 0;//(0.000006f);
  //jacobian(6) *= 0;//(0.000006f);


  std::vector<float> jacs(7, 0);

  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }

  ci::app::console() << "Jacobian = [";
  for (size_t v = 0; v < jacs.size(); ++v){
    ci::app::console() << jacs[v] << ",";
  }
  ci::app::console() << "]" << std::endl;

  current_model->UpdatePose(jacs);

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

  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < model->GetBasePose().GetNumDofs(); ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];

    //actually compute the cost function equation for the degree of freedom in question
      
    float deriv_x =
      ((stereo_camera_->ciExtrinsicRotation().at(0, 0)*(front_intersection_point[2] * dof_derivatives_front[0])) +
      (stereo_camera_->ciExtrinsicRotation().at(0, 1)*(front_intersection_point[2] * dof_derivatives_front[1])) +
      (stereo_camera_->ciExtrinsicRotation().at(0, 2)*(front_intersection_point[2] * dof_derivatives_front[2])))
      -
      ((stereo_camera_->ciExtrinsicRotation().at(2, 0)*(front_intersection_point[0] * dof_derivatives_front[0])) +
      (stereo_camera_->ciExtrinsicRotation().at(2, 1)*(front_intersection_point[0] * dof_derivatives_front[1])) +
      (stereo_camera_->ciExtrinsicRotation().at(2, 2)*(front_intersection_point[0] * dof_derivatives_front[2])));

    deriv_x = (dsdf_dx * fx * z_inv_sq_front) * deriv_x;


    float deriv_y =
      ((stereo_camera_->ciExtrinsicRotation().at(1, 0)*(front_intersection_point[2] * dof_derivatives_front[0])) +
      (stereo_camera_->ciExtrinsicRotation().at(1, 1)*(front_intersection_point[2] * dof_derivatives_front[1])) +
      (stereo_camera_->ciExtrinsicRotation().at(1, 2)*(front_intersection_point[2] * dof_derivatives_front[2])))
      -
      ((stereo_camera_->ciExtrinsicRotation().at(2, 0)*(front_intersection_point[1] * dof_derivatives_front[0])) +
      (stereo_camera_->ciExtrinsicRotation().at(2, 1)*(front_intersection_point[1] * dof_derivatives_front[1])) +
      (stereo_camera_->ciExtrinsicRotation().at(2, 2)*(front_intersection_point[1] * dof_derivatives_front[2])));

    deriv_y = (dsdf_dy * fy * z_inv_sq_front) * deriv_y;

    float pval = DeltaFunction(sdf) * (deriv_x + deriv_y);

    jacobian(dof) += region_agreement * pval;

  }

}

void ComputeAreas(cv::Mat &sdf, size_t &fg_area, size_t &bg_area, size_t &contour_area){

  fg_area = bg_area = 0;

  for (auto r = 0; r < sdf.rows; ++r){
    for (auto c = 0; c < sdf.rows; ++c){

      if (sdf.at<float>(r, c) <= float(3) - 1e-1 && sdf.at<float>(r, c) >= -float(3) + 1e-1)
        contour_area++;

      fg_area += sdf.at<float>(r, c);
      bg_area += (1.0f - sdf.at<float>(r, c));
    }
  }

}

void StereoPWP3D::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){

  cv::Mat sdf_image, front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, sdf_image, front_intersection_image, back_intersection_image);

  size_t fg_area, bg_area = 0;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  ci::app::console() << "Contour area = " << contour_area << std::endl;

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //-log(H * P_f + (1-H) * P_b)
        error += GetErrorValue(r, c, sdf_im_data[i], 1.0f);

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

//
//
//cv::Mat left_to_right_projection(left_front_intersection_image.size(), CV_32FC3), right_to_left_projection(left_front_intersection_image.size(), CV_32FC3);
//cv::Mat left_to_right_projected(left_front_intersection_image.size(), CV_32FC1), right_to_left_projected(left_front_intersection_image.size(), CV_32FC1);
//
//for (int r = 0; r < left_front_intersection_image.rows; ++r){
//  for (int c = 0; c < left_back_intersection_image.cols; ++c){
//
//    const cv::Vec3f &left_point = left_front_intersection_image.at<cv::Vec3f>(r, c);
//    const cv::Vec3f &right_point = right_front_intersection_image.at<cv::Vec3f>(r, c);
//
//    const ci::Vec3f right_point_in_left = stereo_camera_->TransformPointFromRightToLeft(ci::Vec3f(right_point[0], right_point[1], right_point[2]));
//    const ci::Vec3f right_back_again = stereo_camera_->TransformPointFromLeftToRight(right_point_in_left);
//    const ci::Vec3f right_point2(right_point[0], right_point[1], right_point[2]);
//    const float right_dist = (right_point2 - right_back_again).dot((right_point2 - right_back_again));
//
//    const ci::Vec3f left_point_in_right = stereo_camera_->TransformPointFromLeftToRight(ci::Vec3f(left_point[0], left_point[1], left_point[2]));
//    const ci::Vec3f left_back_again = stereo_camera_->TransformPointFromRightToLeft(left_point_in_right);
//    const ci::Vec3f left_point2(left_point[0], left_point[1], left_point[2]);
//    const float left_dist = (left_point2 - left_back_again).dot((left_point2 - left_back_again));
//
//    ci::app::console() << "Left dist = " << left_dist << " and Right dist = " << right_dist << std::endl;
//
//    left_to_right_projection.at<cv::Vec3f>(r, c) = cv::Vec3f(left_point_in_right[0], left_point_in_right[1], left_point_in_right[2]);
//    right_to_left_projection.at<cv::Vec3f>(r, c) = cv::Vec3f(right_point_in_left[0], right_point_in_left[1], right_point_in_left[2]);
//
//    auto projected_from_right = stereo_camera_->right_eye()->ProjectPointToPixel(left_point_in_right);
//    bool x_good = projected_from_right.x >= 0 && projected_from_right.x < left_to_right_projected.cols;
//    bool y_good = projected_from_right.y >= 0 && projected_from_right.y < left_to_right_projected.rows;
//    if (x_good && y_good)
//      left_to_right_projected.at<float>(projected_from_right.y, projected_from_right.x) = left_point_in_right[2];
//
//    auto projected_from_left = stereo_camera_->left_eye()->ProjectPointToPixel(right_point_in_left);
//    x_good = projected_from_left.x >= 0 && projected_from_left.x < left_to_right_projected.cols;
//    y_good = projected_from_left.y >= 0 && projected_from_left.y < left_to_right_projected.rows;
//    if (x_good && y_good)
//      right_to_left_projected.at<float>(projected_from_left.y, projected_from_left.x) = right_point_in_left[2];
//
//  }
//}