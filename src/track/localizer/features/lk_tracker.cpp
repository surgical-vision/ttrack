#include <cinder/app/App.h>

#include "../../../../include/ttrack/track/localizer/features/lk_tracker.hpp"
#include "../../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

LKTracker::LKTracker(boost::shared_ptr<MonocularCamera> camera) : camera_(camera) {
  win_size_ = cv::Size(31, 31);
  term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
}

void LKTracker::TrackLocalPoints(cv::Mat &current_frame){

  double t = (double)cv::getTickCount();

  cv::cvtColor(current_frame, current_frame_gray_, CV_BGR2GRAY);

  //points_test[0] = GetPointsOnPreviousImage();
   
  std::vector<unsigned char> status;
  std::vector<float> err;

  cv::calcOpticalFlowPyrLK(previous_frame_gray_, current_frame_gray_, points_test[0], points_test[1], status, err, win_size_, 3, term_crit_, 0, 0.001);

  for (size_t i = 0; i < points_test[1].size(); ++i){

    if (status[i] == 0){
      tracked_points_[i].found_image_point = cv::Vec2f(-1, -1);
      continue;
    }

    //do something with err?
    tracked_points_[i].found_image_point = points_test[1][i];
  
  }

  std::swap(points_test[0], points_test[1]);

  previous_frame_gray_ = current_frame_gray_.clone();

  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  //ci::app::console() << "Processing LK time = " << t << std::endl;

}

void LKTracker::UpdatePointsOnModelBeforeDerivatives(const Pose &pose){
  
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


void LKTracker::UpdatePointsOnModelAfterDerivatives(const Pose &pose){
  
  for (size_t i = 0; i < tracked_points_.size(); ++i){

    if (tracked_points_[i].frame_point[1] >= 0 && tracked_points_[i].frame_point[0] >= 0){

      cv::Vec3f in_camera_coords = pose.TransformPoint(tracked_points_[i].model_point);
      cv::Point2f on_image_plane = camera_->ProjectPoint(cv::Point3f(in_camera_coords));
      tracked_points_[i].frame_point = on_image_plane;
    }

  }

}


std::vector<float> LKTracker::GetDerivativesForPoints(const Pose &pose) {

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

std::vector<float> LKTracker::GetPointDerivative(const cv::Vec3f &world_previous, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose){

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

void LKTracker::InitializeTracker(cv::Mat &current_frame, const Pose &pose){

  const cv::Size subPixWinSize(10, 10);
   
  cv::Mat gray;
  cv::cvtColor(current_frame, gray, CV_BGR2GRAY);

  //std::vector<cv::Point2f> points;
  cv::goodFeaturesToTrack(gray, points_test[0], 50, 0.01, 10, CreateMask(), 3, 0, 0.04);
  cv::cornerSubPix(gray, points_test[0], subPixWinSize, cv::Size(-1, -1), term_crit_);

  tracked_points_.clear();
  //for (int i = 0; i < points_test[0].size(); ++i){
  //  tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(0, 0)));
  //}
  for (size_t i = 0; i < points_test[0].size(); ++i){
    cv::Vec3f &point_on_model = front_intersection_image_.at<cv::Vec3f>(points_test[0][i].y, points_test[0][i].x);
    if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){
      tracked_points_.push_back(TrackedPoint(cv::Vec3f(0,0,0), cv::Vec2f(-1,-1)));
    }
    else{
      tracked_points_.push_back(TrackedPoint(pose.InverseTransformPoint(point_on_model), points_test[0][i]));
    }
  }
 
  previous_frame_gray_ = gray.clone();

}

cv::Mat LKTracker::CreateMask(){

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

std::vector<cv::Point2f> LKTracker::GetPointsOnPreviousImage(){
  std::vector<cv::Point2f> pts;
  for (auto pt : tracked_points_){
    if (pt.frame_point != cv::Vec2f(-1, -1))
      pts.push_back(pt.found_image_point);
  }
  return pts;
}