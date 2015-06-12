#include <cinder/app/App.h>

#include "../../../../include/ttrack/track/localizer/features/feature_localizer.hpp"
#include "../../../../include/ttrack/constants.hpp"

using namespace ttrk;

FeatureLocalizer::FeatureLocalizer(boost::shared_ptr<MonocularCamera> camera) : camera_(camera) {
 
}


void FeatureLocalizer::UpdatePointsOnModelBeforeDerivatives(const Pose &pose){

  for (size_t i = 0; i < tracked_points_.size(); ++i){

    if (tracked_points_[i].frame_point[1] >= 0 && tracked_points_[i].frame_point[0] >= 0){

      if (tracked_points_[i].frame_point[1] > front_intersection_image_.rows || tracked_points_[i].frame_point[0] >= front_intersection_image_.cols){
      }
      else{
        cv::Vec3f pt = front_intersection_image_.at<cv::Vec3f>(tracked_points_[i].frame_point[1], tracked_points_[i].frame_point[0]);
        if (pt[0] == GL_FAR && pt[1] == GL_FAR && pt[2] == pt[2]){
          tracked_points_[i].frame_point = cv::Vec2i(-1, -1);
        }
        else{
          tracked_points_[i].model_point = pose.InverseTransformPoint(pt);
        }
      }

    }
  }
}


void FeatureLocalizer::UpdatePointsOnModelAfterDerivatives(const Pose &pose){

  for (size_t i = 0; i < tracked_points_.size(); ++i){

    if (tracked_points_[i].frame_point[1] >= 0 && tracked_points_[i].frame_point[0] >= 0){

      cv::Vec3f in_camera_coords = pose.TransformPoint(tracked_points_[i].model_point);
      cv::Point2f on_image_plane = camera_->ProjectPoint(cv::Point3f(in_camera_coords));
      tracked_points_[i].frame_point = on_image_plane;
    }

  }

}


std::vector<float> FeatureLocalizer::GetDerivativesForPoints(const Pose &pose) {

  //UpdatePointsOnModelBeforeDerivatives(pose);

  current_error_ = 0;
  size_t num_pts = tracked_points_.size();

  std::vector<float> ret;
  for (int i = 0; i < 7; ++i){
    ret.push_back(0.0f);
  }

  for (auto tp : tracked_points_){

    if (tp.frame_point[0] == -1)
      continue;
    std::vector<float> pds = GetPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, pose);

    for (int i = 0; i < pds.size(); ++i){
      ret[i] += pds[i];
    }

  }

  return ret;

}

std::vector<float> FeatureLocalizer::GetPointDerivative(const cv::Vec3f &world_previous, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose){

  cv::Vec3f camera_coordinates = pose.TransformPoint(world_previous);
  if (camera_coordinates[2] == 0.0) camera_coordinates[2] = 0.001;
  float z_inv_sq = 1.0 / camera_coordinates[2];

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

    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const float dXdL = camera_->Fx() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[0]) - (camera_coordinates[0] * dof_derivatives[2])));
    const float dYdL = camera_->Fy() * (z_inv_sq*((camera_coordinates[2] * dof_derivatives[1]) - (camera_coordinates[1] * dof_derivatives[2])));
    const float inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
    const float dPxdL = -2 * x_error * dXdL;
    const float dPydL = -2 * y_error * dYdL;
    jacobians[dof] = (inv_sqrt * (dPxdL + dPydL));

  }

  current_error_ += std::sqrt(x_error * x_error + y_error * y_error);

  return jacobians;


}

cv::Mat FeatureLocalizer::CreateMask(){
  cv::Mat mask = cv::Mat::zeros(front_intersection_image_.size(), CV_8UC1);
  for (int r = 0; r < mask.rows; ++r){
    for (int c = 0; c < mask.cols; ++c){
      const cv::Vec3f &f = front_intersection_image_.at<cv::Vec3f>(r, c);
      if (f[0] == GL_FAR && f[1] == GL_FAR && f[2] == GL_FAR){
        continue;
      }
      mask.at<unsigned char>(r, c) = 255;
    }
  }
  return mask;
}

std::vector<cv::Point2f> FeatureLocalizer::GetPointsOnPreviousImage(){
  std::vector<cv::Point2f> pts;
  for (auto pt : tracked_points_){
    if (pt.frame_point != cv::Vec2f(-1, -1))
      pts.push_back(pt.found_image_point);
  }
  return pts;
}
