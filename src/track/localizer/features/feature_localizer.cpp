#include <cinder/app/App.h>

#include "../../../../include/ttrack/track/localizer/features/feature_localizer.hpp"
#include "../../../../include/ttrack/constants.hpp"
#include "../../../../include/ttrack/track/model/model.hpp"
#include <ttrack/track/localizer/localizer.hpp>
#include <ttrack/utils/helpers.hpp>

using namespace ttrk;

const int point_threshold = 25;

FeatureLocalizer::FeatureLocalizer(boost::shared_ptr<MonocularCamera> camera) : camera_(camera), frame_count_(0) {
 
}

std::vector<float> FeatureLocalizer::GetArticulatedDerivativesForPoints(boost::shared_ptr<Model> current_model, const cv::Mat &articulated_index_image){

  current_error_ = 0;
  size_t num_pts = current_model->mps.tracked_points_.size();

  std::vector<float> ret;
  for (int i = 0; i < current_model->GetNumberOfDofs(); ++i){
    ret.push_back(0.0f);
  }

  for (size_t i = 0; i < current_model->mps.tracked_points_.size(); ++i){

    auto &tp = current_model->mps.tracked_points_[i];
    
    if (tp.point_in_view == false) continue;
    if (tp.point_tracked_on_model == false) continue;

    if (!cv::Rect(0, 0, articulated_index_image.cols, articulated_index_image.rows).contains(cv::Point(tp.frame_point[0], tp.frame_point[1]))) {
      tp.point_in_view = false;
      tp.point_tracked_on_model = false;
      continue;
    }

    unsigned char index = articulated_index_image.at<unsigned char>(tp.frame_point[1], tp.frame_point[0]);

    if (index == 255){
      tp.point_tracked_on_model = false;
      continue;
    }

    //if (index >= 3) continue;

    if (l2_distance(tp.frame_point, tp.found_image_point) > point_threshold)
      continue;
    
    std::vector<float> pds = GetArticulatedPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, current_model, index);

    //if (pds[8] != 0.0f || pds[9] != 0.0f | pds[10] != 0.0f){
    //  ci::app::console() << "ERror!!!!" << std::endl;
    //}

    for (int i = 0; i < pds.size(); ++i){
      ret[i] += pds[i];
    }

  }

  return ret;


}



std::vector<float> FeatureLocalizer::GetDerivativesForPoints(boost::shared_ptr<Model> current_model, const Pose &pose) {

  current_error_ = 0;
  size_t num_pts = current_model->mps.tracked_points_.size();

  std::vector<float> ret;
  for (int i = 0; i < 7; ++i){
    ret.push_back(0.0f);
  }

  std::size_t number_of_tracked_points = 0;

  for (auto tp : current_model->mps.tracked_points_){

    if (tp.point_in_view == false) continue;
    if (tp.point_tracked_on_model == false) continue;

    if (l2_distance(tp.frame_point, tp.found_image_point) > point_threshold)
      continue;

    int border_y = 0.1 * current_model->mps.current_frame.rows;
    int border_x = 0.1 * current_model->mps.current_frame.cols;

    if (cv::Rect(0, 0, current_model->mps.current_frame.cols, current_model->mps.current_frame.rows).contains(cv::Point(tp.found_image_point)) &&
      !cv::Rect(border_x, border_y, current_model->mps.current_frame.cols - (2 * border_x), current_model->mps.current_frame.rows - (2 * border_y)).contains(cv::Point(tp.found_image_point))){
      tp.point_in_view = false;
      tp.point_tracked_on_model = false;
      continue;
    }

    std::vector<float> pds = GetPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, pose);
    number_of_tracked_points++;

    for (int i = 0; i < pds.size(); ++i){
      ret[i] += pds[i];
    }

  }

  return ret;

}

std::vector<float> FeatureLocalizer::GetDerivativesForPointsOnRigidBody(boost::shared_ptr<Model> current_model, const Pose &pose, const cv::Mat &index_image) {

  // THIS IS USED IN THE ARTICULATED TRACKER ONLY!
  
  current_error_ = 0;
  size_t num_pts = current_model->mps.tracked_points_.size();

  std::vector<float> ret;
  for (int i = 0; i < 7; ++i){
    ret.push_back(0.0f);
  }

  std::size_t number_of_tracked_points = 0;

  for (auto &tp : current_model->mps.tracked_points_){

    //if (tp.frame_point[0] == -1) continue;

    if (tp.point_in_view == false) continue;
    if (tp.point_tracked_on_model == false) continue;

    if (!cv::Rect(0, 0, index_image.cols, index_image.rows).contains(cv::Point(tp.frame_point[0], tp.frame_point[1]))) {
      tp.point_in_view = false;
      tp.point_tracked_on_model = false;
      continue;
    }

    unsigned char index = index_image.at<unsigned char>(tp.frame_point[1], tp.frame_point[0]);

    if (index != 0) continue; // this must be here as GetPointDerivative just gets the jacobian from Pose not articulated pose.

    if (index == 255){
      ci::app::console() << "This should not happen!" << std::endl;
      tp.point_tracked_on_model = false;
      continue;
    }

    //if (index >= 2) continue;

    if (l2_distance(tp.frame_point, tp.found_image_point) > point_threshold)
      continue;

    std::vector<float> pds = GetPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, pose);

    number_of_tracked_points++;

    for (int i = 0; i < pds.size(); ++i){
      ret[i] += pds[i];
    }

  }

  ci::app::console() << "Tracked " << number_of_tracked_points << " points using LK" << std::endl;

  return ret;

}

bool FeatureLocalizer::NeedsReset() const {

  return (frame_count_ % 10) == 0;


}


void FeatureLocalizer::UpdatePointsOnArticulatedModelAfterDerivatives(boost::shared_ptr<Model> current_model, const cv::Mat &articulated_index_image){
  
  float average_error = 0.0f;

  for (size_t i = 0; i < current_model->mps.tracked_points_.size(); ++i){

    //if (current_model->mps.tracked_points_[i].frame_point[1] >= 0 && current_model->mps.tracked_points_[i].frame_point[0] >= 0){

    if (!current_model->mps.tracked_points_[i].point_tracked_on_model) continue;
    if (!current_model->mps.tracked_points_[i].point_in_view) continue;

    if (!cv::Rect(0, 0, articulated_index_image.cols, articulated_index_image.rows).contains(cv::Point(current_model->mps.tracked_points_[i].frame_point[0], current_model->mps.tracked_points_[i].frame_point[1]))) {
      current_model->mps.tracked_points_[i].point_in_view = false;
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
    }


    auto index = (size_t)articulated_index_image.at<unsigned char>(current_model->mps.tracked_points_[i].frame_point[1], current_model->mps.tracked_points_[i].frame_point[0]);
    if (index == 255){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;;
      continue;
    }

    cv::Vec3f in_camera_coords = current_model->GetComponentPose(index).TransformPoint(current_model->mps.tracked_points_[i].model_point);
    cv::Point2f on_image_plane = camera_->ProjectPoint(cv::Point3f(in_camera_coords));

    auto idx = (size_t)articulated_index_image.at<unsigned char>(current_model->mps.tracked_points_[i].frame_point[1], current_model->mps.tracked_points_[i].frame_point[0]);
    if (idx != current_model->mps.tracked_points_[i].component_idx){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;;
      continue;
    }

    if (!cv::Rect(0, 0, current_model->mps.previous_frame.cols, current_model->mps.previous_frame.rows).contains(on_image_plane)){
      current_model->mps.tracked_points_[i].point_in_view = false;
      continue;
    }
    else{
      current_model->mps.tracked_points_[i].point_in_view = true;
    }

    current_model->mps.tracked_points_[i].frame_point = on_image_plane;

    average_error += l2_distance(cv::Vec2f(on_image_plane), cv::Vec2f(current_model->mps.tracked_points_[i].found_image_point));

    cv::line(current_model->debug_info.tracked_feature_points, cv::Point(current_model->mps.tracked_points_[i].frame_point), cv::Point(current_model->mps.tracked_points_[i].found_image_point), cv::Scalar(0, 255, 0));

    //}

  }

  if (current_model->mps.tracked_points_.size())
    ci::app::console() << "Average error from feature localizer: " << average_error / current_model->mps.tracked_points_.size() << std::endl;

}


void FeatureLocalizer::UpdatePointsOnModelAfterDerivatives(boost::shared_ptr<Model> current_model, const Pose &pose){

  cv::Mat &m = current_model->debug_info.tracked_feature_points;

  for (size_t i = 0; i < current_model->mps.tracked_points_.size(); ++i){

    //if (current_model->mps.tracked_points_[i].frame_point[1] >= 0 && current_model->mps.tracked_points_[i].frame_point[0] >= 0){

    if (!current_model->mps.tracked_points_[i].point_tracked_on_model) continue;
    if (!current_model->mps.tracked_points_[i].point_in_view) continue; 

    cv::Vec3f in_camera_coords = pose.TransformPoint(current_model->mps.tracked_points_[i].model_point);
    cv::Point2f on_image_plane = camera_->ProjectPoint(cv::Point3f(in_camera_coords));

    if (!cv::Rect(0, 0, current_model->mps.previous_frame.cols, current_model->mps.previous_frame.rows).contains(on_image_plane)){
      current_model->mps.tracked_points_[i].frame_point = on_image_plane;
      current_model->mps.tracked_points_[i].point_in_view = false;
      continue;
    }
    else{
      current_model->mps.tracked_points_[i].point_in_view = true;
    }

    current_model->mps.tracked_points_[i].frame_point = on_image_plane;

    cv::circle(current_model->debug_info.tracked_feature_points, cv::Point(current_model->mps.tracked_points_[i].frame_point), 1, cv::Scalar(0, 255, 0));

   //}

  }

}






std::vector<float> FeatureLocalizer::GetPointDerivative(const cv::Vec3f &world_previous, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose){

  cv::Vec3f camera_coordinates = pose.TransformPoint(world_previous);
  if (camera_coordinates[2] == 0.0) camera_coordinates[2] = 0.001;
  float z_inv_sq = 1.0 / (camera_coordinates[2] * camera_coordinates[2]);

  std::vector<ci::Vec3f> jacs = pose.ComputeJacobian(ci::Vec3f(camera_coordinates[0], camera_coordinates[1], camera_coordinates[2]));

  std::vector<float> jacobians;
  for (int dof = 0; dof < pose.GetNumDofs(); ++dof){
    jacobians.push_back(0.0f);
  }

  const float x_error = image_new[0] - image_previous[0];
  const float y_error = image_new[1] - image_previous[1];

  if (std::abs(x_error) < std::numeric_limits<float>::epsilon() && std::abs(y_error) < std::numeric_limits<float>::epsilon())
    return jacobians;

  for (int dof = 0; dof<pose.GetNumDofs(); dof++){

    //if (dof < 3) continue;

    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const float dXdL = camera_->Fx() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[0]) - (camera_coordinates[0] * dof_derivatives[2])));
    const float dYdL = camera_->Fy() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[1]) - (camera_coordinates[1] * dof_derivatives[2])));
    const float inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
    const float dPxdL = -2 * x_error * dXdL;
    const float dPydL = -2 * y_error * dYdL;
    //jacobians[dof] = (inv_sqrt * (dPxdL + dPydL));
    jacobians[dof] = (dPxdL + dPydL);

  }

  current_error_ += std::sqrt(x_error * x_error + y_error * y_error);

  return jacobians;


}


float FeatureLocalizer::GetPointProjectionError(boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, const cv::Mat &articulated_component_image){

  cv::Mat debug_frame = current_model->mps.current_frame.clone();

  float error = 0.0f;

  for (size_t i = 0; i < current_model->mps.tracked_points_.size(); ++i){

    auto &tp = current_model->mps.tracked_points_[i];

    if (!current_model->mps.tracked_points_[i].point_tracked_on_model) continue;
    if (!current_model->mps.tracked_points_[i].point_in_view) continue;

    unsigned char index = articulated_component_image.at<unsigned char>(tp.frame_point[1], tp.frame_point[0]);

    if (index == 255){
      continue;
    }

    cv::Vec3f camera_coordinates = current_model->GetComponentPose(index).TransformPoint(tp.model_point);
    cv::Point2f pixel = camera->ProjectPointToPixel(cv::Point3f(camera_coordinates));
    cv::circle(debug_frame, pixel, 3, cv::Scalar(0, 255, 0), 2);
    cv::circle(debug_frame, cv::Point2f(tp.found_image_point), 3, cv::Scalar(255, 255, 0), 2);

    error += l2_distance(cv::Vec2d(cv::Vec2f(pixel)), cv::Vec2d(tp.found_image_point));

  }

  return error;

}

std::vector<float> FeatureLocalizer::GetArticulatedPointDerivative(const cv::Vec3f &model_previous, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const boost::shared_ptr<Model> current_model, const size_t articulated_component_idx){

  cv::Vec3f camera_coordinates = current_model->GetComponentPose(articulated_component_idx).TransformPoint(model_previous);
  if (camera_coordinates[2] == 0.0) camera_coordinates[2] = 0.001;
  float z_inv_sq = 1.0 / (camera_coordinates[2] * camera_coordinates[2]);

  //get the pose derivatives
  std::vector<ci::Vec3f> jacs = current_model->ComputeJacobian(ci::Vec3f(camera_coordinates[0], camera_coordinates[1], camera_coordinates[2]), articulated_component_idx);

  std::vector<float> jacobians;
  for (int dof = 0; dof < current_model->GetNumberOfDofs(); ++dof){
    jacobians.push_back(0.0f);
  }

  const float x_error = image_new[0] - image_previous[0];
  const float y_error = image_new[1] - image_previous[1];

  if (std::abs(x_error) < std::numeric_limits<float>::epsilon() && std::abs(y_error) < std::numeric_limits<float>::epsilon())
    return jacobians;

  for (int dof = 0; dof < current_model->GetNumberOfDofs(); dof++){

    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const float dXdL = camera_->Fx() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[0]) - (camera_coordinates[0] * dof_derivatives[2])));
    const float dYdL = camera_->Fy() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[1]) - (camera_coordinates[1] * dof_derivatives[2])));
    const float inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
    const float dPxdL = -2 * x_error * dXdL;
    const float dPydL = -2 * y_error * dYdL;
    //jacobians[dof] = (inv_sqrt * (dPxdL + dPydL));
    jacobians[dof] = (dPxdL + dPydL);

  }

  current_error_ += std::sqrt(x_error * x_error + y_error * y_error);

  return jacobians;



}



std::vector<float> FeatureLocalizer::GetResiduals(boost::shared_ptr<Model> current_model) const{

  std::vector<float> res;
  for (auto tp : current_model->mps.tracked_points_){

    if (tp.frame_point[0] == -1)
      continue;

    float sq_error = (tp.found_image_point[0] - tp.frame_point[0])*(tp.found_image_point[0] - tp.frame_point[0]) + (tp.found_image_point[1] - tp.frame_point[1])*(tp.found_image_point[1] - tp.frame_point[1]);

    float error = 0;
    //float error = (tp.found_image_point[0] - tp.frame_point[0]) + (tp.found_image_point[1] - tp.frame_point[1]);
    if (sq_error > 0)
      error = std::sqrt((float)sq_error);

    res.push_back(error);

  }

  return res;


}


std::vector<float> FeatureLocalizer::GetIntensityDerivative(const cv::Vec3f &camera_coordinates_, const float &dx, const float &dy, const Pose &pose){

  cv::Vec3f camera_coordinates = camera_coordinates_;

  if (camera_coordinates[2] == 0.0) camera_coordinates[2] = 0.001;
  float z_inv_sq = 1.0 / (camera_coordinates[2] * camera_coordinates[2]);

  std::vector<ci::Vec3f> jacs = pose.ComputeJacobian(ci::Vec3f(camera_coordinates[0], camera_coordinates[1], camera_coordinates[2]));

  std::vector<float> jacobians;
  for (int dof = 0; dof < pose.GetNumDofs(); ++dof){

    jacobians.push_back(0.0f);
    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const float dXdL = camera_->Fx() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[0]) - (camera_coordinates[0] * dof_derivatives[2])));
    const float dYdL = camera_->Fy() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[1]) - (camera_coordinates[1] * dof_derivatives[2])));

    jacobians[dof] = dx * dXdL + dy * dYdL;

  }

  return jacobians;

}

cv::Mat FeatureLocalizer::GetJacobianMatrix(boost::shared_ptr<Model> current_model, const Pose &pose) {

  size_t number_tracked_points = 0;
  for (auto tp : current_model->mps.tracked_points_){
    if (tp.frame_point[0] == -1)
      continue;
    number_tracked_points++;
  }

  cv::Mat jacobian = cv::Mat::zeros(number_tracked_points, 7, CV_32FC1);

  for (size_t i = 0, r = 0; i < current_model->mps.tracked_points_.size(); ++i){

    auto &tp = current_model->mps.tracked_points_[i];

    if (tp.frame_point[0] == -1)
      continue;

    std::vector<float> pds = GetPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, pose);

    for (size_t c = 0; c < jacobian.cols; ++c){

      jacobian.at<float>(r, c) = pds[c];

    }

    ++r;
  }

  return jacobian;


}

cv::Mat FeatureLocalizer::CreateMask(boost::shared_ptr<Model> current_model){
  cv::Mat mask = cv::Mat::zeros(current_model->mps.front_intersection_image_.size(), CV_8UC1);
  for (int r = 0; r < mask.rows; ++r){
    for (int c = 0; c < mask.cols; ++c){
      const cv::Vec3f &f = current_model->mps.front_intersection_image_.at<cv::Vec3f>(r, c);
      if (f[0] == GL_FAR && f[1] == GL_FAR && f[2] == GL_FAR){
        continue;
      }

      if (Localizer::occlusion_image.at<float>(r, c) < (f[2] - 0.1)){
        continue;
      }

      mask.at<unsigned char>(r, c) = 255;
    }
  }
  return mask;
}

std::vector<cv::Point2f> FeatureLocalizer::GetPointsOnPreviousImage(boost::shared_ptr<Model> current_model){
  std::vector<cv::Point2f> pts;
  for (auto pt : current_model->mps.tracked_points_){
    if (pt.frame_point != cv::Vec2f(-1, -1))
      pts.push_back(pt.found_image_point);
  }
  return pts;
}
