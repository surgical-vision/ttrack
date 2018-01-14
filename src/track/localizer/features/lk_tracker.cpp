#include <cinder/app/App.h>

#include "../../../../include/ttrack/track/localizer/features/lk_tracker.hpp"
#include "../../../../include/ttrack/utils/helpers.hpp"
#include <ttrack/track/localizer/localizer.hpp>

using namespace ttrk;

LKTracker::LKTracker(boost::shared_ptr<MonocularCamera> camera) : FeatureLocalizer(camera) {
  win_size_ = cv::Size(31, 31);
  term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
}

void LKTracker::TrackLocalPoints(cv::Mat &current_frame, boost::shared_ptr<Model> current_model){

  cv::cvtColor(current_frame, current_model->mps.current_frame, CV_BGR2GRAY);

  std::vector<unsigned char> status;
  std::vector<float> err;

  if (current_model->mps.points_test[0].empty()) return;

  if (current_model->mps.points_test[0].size() != current_model->mps.tracked_points_.size()){
    current_model->mps.is_initialised = false;
    current_model->mps.points_test[0].clear();
    current_model->mps.points_test[1].clear();
    current_model->mps.tracked_points_.clear();
    return;
  }

  cv::calcOpticalFlowPyrLK(current_model->mps.previous_frame, current_model->mps.current_frame, current_model->mps.points_test[0], current_model->mps.points_test[1], status, err, win_size_, 3, term_crit_, 0, 0.001);
 
  cv::Mat &x1 = current_model->mps.previous_frame;
  cv::Mat &x2 = current_model->mps.current_frame;

  cv::Mat &x3 = current_model->mps.front_intersection_image_;

  current_model->debug_info.tracked_feature_points = current_frame.clone();
  
  cv::Mat &m = current_model->debug_info.tracked_feature_points;

  size_t number_of_tracked_points = 0;

  for (size_t i = 0; i < current_model->mps.points_test[1].size(); ++i){

    if (current_model->mps.points_test[1][i].x < 5 || current_model->mps.points_test[1][i].y < 5 || current_model->mps.points_test[1][i].x >= (current_frame.cols-5) || current_model->mps.points_test[1][i].y >= (current_frame.rows-5)){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
      //current_model->mps.tracked_points_[i].use_point_this_run = false;
    }

    if (current_model->mps.points_test[0][i].x < 5 || current_model->mps.points_test[0][i].y < 5 || current_model->mps.points_test[0][i].x >= (current_frame.cols-5) || current_model->mps.points_test[0][i].y >= (current_frame.rows-5)){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
      //current_model->mps.tracked_points_[i].use_point_this_run = false;
    }

    if (status[i] == 0){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
      //current_model->mps.tracked_points_[i].use_point_this_run = false;
    }

    if (current_model->mps.front_intersection_image_.at<cv::Vec3f>(current_model->mps.points_test[0][i])[0] == GL_FAR){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
      //current_model->mps.tracked_points_[i].use_point_this_run = false;
    }

    if (current_model->mps.front_intersection_image_.at<cv::Vec3f>(current_model->mps.points_test[1][i])[0] == GL_FAR){
      current_model->mps.tracked_points_[i].point_tracked_on_model = false;
      continue;
      //current_model->mps.tracked_points_[i].use_point_this_run = false;
    }

    int start_r = current_model->mps.points_test[1][i].y - (win_size_.height / 2);
    int start_c = current_model->mps.points_test[1][i].x - (win_size_.width / 2);

    if (start_r < 0) start_r = 0;
    if (start_c < 0) start_c = 0;

    int end_r = current_model->mps.points_test[1][i].y + (win_size_.height / 2);
    int end_c = current_model->mps.points_test[1][i].x + (win_size_.width / 2);
    if (end_r >= current_frame.rows) end_r = current_frame.rows - 1;
    if (end_c >= current_frame.cols) end_c = current_frame.cols - 1;

    bool should_continue = false;
    for (int r = start_r; r < end_r; ++r){
      for (int c = start_c; c < end_c; ++c){
        //if (Localizer::occlusion_image.at<float>(current_model->mps.points_test[1][i]) < (current_model->mps.front_intersection_image_.at<cv::Vec3f>(current_model->mps.points_test[1][i])[2] - 0.1)) {
        if (Localizer::occlusion_image.at<float>(r,c) < (current_model->mps.front_intersection_image_.at<cv::Vec3f>(r, c)[2] - 0.1)) {
          current_model->mps.tracked_points_[i].point_tracked_on_model = false;
          should_continue = true;
          break;
        }
      }
      if (should_continue) break;
    }
    if (should_continue) continue;        

    cv::circle(current_model->debug_info.tracked_feature_points, current_model->mps.points_test[0][i], 2, cv::Scalar(255, 0, 0));
    cv::circle(current_model->debug_info.tracked_feature_points, current_model->mps.points_test[1][i], 2, cv::Scalar(0, 0, 255));


    //do something with err?
    current_model->mps.tracked_points_[i].found_image_point = current_model->mps.points_test[1][i];
    current_model->mps.tracked_points_[i].point_tracked_on_model = true;

    number_of_tracked_points++;

  }

  if (number_of_tracked_points < 3) {
    current_model->mps.is_initialised = false;
  }

  std::swap(current_model->mps.points_test[0], current_model->mps.points_test[1]);

  current_model->mps.previous_frame = current_model->mps.current_frame.clone();

}

void LKTracker::InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model){

  const cv::Size subPixWinSize(10, 10);
   
  cv::Mat gray;
  if (current_frame.type() == CV_8UC3)
    cv::cvtColor(current_frame, gray, CV_BGR2GRAY);
  else if (current_frame.type() == CV_8UC1)
    gray = current_frame.clone();


  current_model->mps.points_test[0].clear();
  current_model->mps.points_test[1].clear();

  //std::vector<cv::Point2f> points;
  cv::goodFeaturesToTrack(gray, current_model->mps.points_test[0], 50, 0.01, 10, CreateMask(current_model), 3, 0, 0.04);
  cv::cornerSubPix(gray, current_model->mps.points_test[0], subPixWinSize, cv::Size(-1, -1), term_crit_);

  current_model->mps.tracked_points_.clear();
  
  for (size_t i = 0; i < current_model->mps.points_test[0].size(); ++i){

    cv::Point rounded(std::round(current_model->mps.points_test[0][i].x), std::round(current_model->mps.points_test[0][i].y));
    if (!cv::Rect(0, 0, current_frame.cols, current_frame.rows).contains(rounded)) {
      current_model->mps.tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));
      current_model->mps.tracked_points_.back().point_in_view = false;
      current_model->mps.tracked_points_.back().point_tracked_on_model = false;
      continue;
    }

    cv::Vec3f &point_on_model = current_model->mps.front_intersection_image_.at<cv::Vec3f>(rounded);
    if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){
      current_model->mps.tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));
      current_model->mps.tracked_points_.back().point_in_view = false;
      current_model->mps.tracked_points_.back().point_tracked_on_model = false;
    }
    else{
      auto pt_in_world_coords = current_model->GetBasePose().InverseTransformPoint(point_on_model);
      auto back_again = current_model->GetBasePose().TransformPoint(pt_in_world_coords);
      current_model->mps.tracked_points_.push_back(TrackedPoint(pt_in_world_coords, current_model->mps.points_test[0][i]));
      current_model->mps.tracked_points_.back().point_tracked_on_model = true;
      current_model->mps.tracked_points_.back().point_in_view = true;

    }
  }
 

  current_model->mps.previous_frame = gray.clone();
  current_model->mps.is_initialised = true;

}

void LKTracker::InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &component_image){

  const cv::Size subPixWinSize(10, 10);

  cv::Mat gray;
  if (current_frame.type() == CV_8UC3)
    cv::cvtColor(current_frame, gray, CV_BGR2GRAY);
  else if (current_frame.type() == CV_8UC1)
    gray = current_frame.clone();
  
  current_model->mps.points_test[0].clear();
  current_model->mps.points_test[1].clear();

  for (int i = 0; i < 6; ++i){
    std::vector<cv::Point2f> points_per_component;

    cv::Mat mask = component_image == i;
    if (i == 2 || i == 3) continue;

    cv::goodFeaturesToTrack(gray, points_per_component, 15, 0.01, 10, mask, 3, 0, 0.04);
    if (points_per_component.empty()) continue;
    cv::cornerSubPix(gray, points_per_component, subPixWinSize, cv::Size(-1, -1), term_crit_);
    current_model->mps.points_test[0].insert(current_model->mps.points_test[0].end(), points_per_component.begin(), points_per_component.end());
  }

  current_model->mps.tracked_points_.clear();
  for (size_t i = 0; i < current_model->mps.points_test[0].size(); ++i){
    cv::Point rounded(std::round(current_model->mps.points_test[0][i].x), std::round(current_model->mps.points_test[0][i].y));
    if (!cv::Rect(0, 0, component_image.cols, component_image.rows).contains(rounded)) {
      current_model->mps.tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));
      current_model->mps.tracked_points_.back().point_in_view = false;
      current_model->mps.tracked_points_.back().point_tracked_on_model = false;
      continue;
    }
    
    cv::Vec3f &point_on_model = current_model->mps.front_intersection_image_.at<cv::Vec3f>(rounded);

    if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){
      current_model->mps.tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));
      current_model->mps.tracked_points_.back().point_in_view = false;
      current_model->mps.tracked_points_.back().point_tracked_on_model = false;
    }
    else{
      unsigned char idx = component_image.at<unsigned char>(rounded);
      auto pt_in_world_coords = current_model->GetComponentPose(idx).InverseTransformPoint(point_on_model);
      auto back_again = current_model->GetComponentPose(idx).TransformPoint(pt_in_world_coords);

      current_model->mps.tracked_points_.push_back(TrackedPoint(current_model->GetComponentPose(idx).InverseTransformPoint(point_on_model), current_model->mps.points_test[0][i]));
      current_model->mps.tracked_points_.back().component_idx = idx;
      current_model->mps.tracked_points_.back().point_tracked_on_model = true;
      current_model->mps.tracked_points_.back().point_in_view = true;

    }
  }

  if (current_model->mps.tracked_points_.size() != current_model->mps.points_test[0].size()){
    int x = 0;
  }

  current_model->mps.previous_frame = gray.clone();
  current_model->mps.is_initialised = true;

}


ArticulatedLKTracker::ArticulatedLKTracker(boost::shared_ptr<MonocularCamera> camera) : LKTracker(camera) {
}


LKTracker3D::LKTracker3D(boost::shared_ptr<MonocularCamera> camera) : LKTracker(camera) {
  term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER, 1, 0.03);
}

void LKTracker3D::SetSpatialDerivatives(boost::shared_ptr<Model> current_model, const Pose &pose){

  const int half_win_size = 15;

  for (auto &pt : current_model->mps.tracked_points_){

    cv::Point proj = camera_->ProjectPointToPixel(cv::Point3d(pose.TransformPoint(pt.model_point)));

    const int start_r = proj.y - half_win_size - 1;
    const int start_c = proj.x - half_win_size - 1;
    const int end_r = proj.y + half_win_size + 1;
    const int end_c = proj.x + half_win_size + 1;

    //REMEMBER WE HAVE A 1 PIXEL BORDER AROUND THE WINDOW SO WE CAN DO DERIVATIVES

    if (start_r < 0 || start_c < 0 || end_r >= current_model->mps.current_frame.rows || end_c >= current_model->mps.current_frame.cols) continue;

    pt.subwindow = current_model->mps.previous_frame(cv::Rect(cv::Point(start_c, start_r), cv::Point(end_c, end_r))).clone();
    pt.surface_at_point = current_model->mps.front_intersection_image_(cv::Rect(cv::Point(start_c, start_r), cv::Point(end_c, end_r))).clone();
    pt.spatial_derivatives.clear();

    cv::Mat &im_t = pt.subwindow;
    cv::Mat &intersection = pt.surface_at_point;
    for (int r = 1; r < im_t.rows - 1; ++r){
      for (int c = 1; c < im_t.cols - 1; ++c){


        const float dx_dl = (float)((float)im_t.at<unsigned char>(r, c + 1) - (float)im_t.at<unsigned char>(r, c - 1)) / (2);
        const float dy_dl = (float)((int)im_t.at<unsigned char>(r + 1, c) - (int)im_t.at<unsigned char>(r - 1, c)) / (2);

        cv::Vec3f model_in_camera_coords = intersection.at<cv::Vec3f>(r, c);
        if (model_in_camera_coords[0] == GL_FAR) continue;
        
        std::vector<float> spatial_derivatives_v = GetIntensityDerivative(model_in_camera_coords, dx_dl, dy_dl, pose);
        pt.spatial_derivatives.push_back(cv::Mat::zeros(7, 1, CV_64FC1));

        for (int k = 0; k < 7; ++k){
          pt.spatial_derivatives.back().at<double>(k) = spatial_derivatives_v[k];
        }
      }
    }
  }



}


std::vector<float> LKTracker3D::GetUpdateStep(boost::shared_ptr<Model> current_model){

  //cv::Mat ti = spatial_derivatives * spatial_derivatives.t();
  //cv::Mat t = (ti).inv(); 

  //cv::Mat identity = ti * t;
  std::vector<float> rr; for (int i = 0; i < 7; ++i) rr.push_back(0.0);
  for (auto pt : current_model->mps.tracked_points_){
    if (pt.spatial_derivatives.size() != pt.temporal_derivatives.size()) {
      ci::app::console() << "Spatial derivative size != temporal derivative size" << std::endl;
      throw std::runtime_error("");
    }

    for (int i = 0; i < pt.spatial_derivatives.size(); ++i){
      cv::Mat &sd = pt.spatial_derivatives[i];
      
      for (int j = 0; j < sd.total(); ++j){
        //negative is applied when we scale it
        rr[j] += 2 * sd.at<double>(j) * pt.temporal_derivatives[i];
      }

    }

  }

  //cv::Mat r = t * temporal_spatial_derivatives;
  //for (size_t i = 0; i < temporal_spatial_derivatives.total(); ++i) rr.push_back(2 * temporal_spatial_derivatives.at<double>(i));
  return rr;

}

void LKTracker3D::TrackLocalPoints(cv::Mat &current_frame, const Pose &pose, boost::shared_ptr<Model> current_model){

  cv::cvtColor(current_frame, current_model->mps.current_frame, CV_BGR2GRAY);

  const int half_win_size = 15;

  for (auto &pt : current_model->mps.tracked_points_){

    cv::Point proj = camera_->ProjectPointToPixel(cv::Point3d(pose.TransformPoint(pt.model_point)));

    const int start_r = proj.y - half_win_size - 1;
    const int start_c = proj.x - half_win_size - 1;
    const int end_r = proj.y + half_win_size + 1;
    const int end_c = proj.x + half_win_size + 1;

    //REMEMBER WE HAVE A 1 PIXEL BORDER AROUND THE WINDOW SO WE CAN DO DERIVATIVES
    pt.temporal_derivatives.clear();

    if (start_r < 0 || start_c < 0 || end_r >= current_frame.rows || end_c >= current_frame.cols) continue;

    cv::Mat subwindow_current = current_model->mps.current_frame(cv::Rect(cv::Point(start_c, start_r), cv::Point(end_c, end_r))).clone();
    cv::Mat &intersection = pt.surface_at_point;

    for (int r = 1; r < subwindow_current.rows - 1; ++r){
      for (int c = 1; c < subwindow_current.cols - 1; ++c){

        cv::Vec3f model_in_camera_coords = intersection.at<cv::Vec3f>(r, c);
        if (model_in_camera_coords[0] == GL_FAR) continue;
        pt.temporal_derivatives.push_back((float)subwindow_current.at<unsigned char>(r, c) - (float)pt.subwindow.at<unsigned char>(r, c));
      }
    }
  }

  return;

  std::vector<unsigned char> status;
  std::vector<float> err;

  if (current_model->mps.points_test[0].empty()) return;

  cv::calcOpticalFlowPyrLK(current_model->mps.previous_frame, current_model->mps.current_frame, current_model->mps.points_test[0], current_model->mps.points_test[1], status, err, win_size_, 3, term_crit_, 0, 0.001);

  std::stringstream ss;
  ss << "c:/tmp/lk_tracker_frame_" << frame_count_ << "_step_" << current_step_index << ".png";
  cv::Mat lk_frame = current_frame.clone();

  for (size_t i = 0; i < current_model->mps.points_test[1].size(); ++i){

    if (status[i] == 0){
      current_model->mps.tracked_points_[i].frame_point = cv::Vec2f(-1, -1);
      continue;
    }

    if (i < 5){

      cv::circle(lk_frame, cv::Point(current_model->mps.points_test[0][i]), 2, cv::Scalar(0, 255, 0));
      cv::circle(lk_frame, cv::Point(current_model->mps.tracked_points_[i].frame_point), 2, cv::Scalar(255, 0, 0));
      cv::circle(lk_frame, cv::Point(current_model->mps.points_test[1][i]), 2, cv::Scalar(255, 255, 0));

    }

    //do something with err?
    current_model->mps.tracked_points_[i].found_image_point = current_model->mps.points_test[1][i];

    ci::app::console() << "Tracked point from " << current_model->mps.tracked_points_[i].frame_point << " --> " << current_model->mps.tracked_points_[i].found_image_point << std::endl;

  }

  cv::imwrite(ss.str(), lk_frame);
  term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER, current_step_index, 0.03);
  current_step_index++;

}


void ArticulalatedLKTrackerFrameToFrame::TrackLocalPoints(cv::Mat &current_frame, boost::shared_ptr<Model> current_model, const cv::Mat &component_image){
  
  if (current_model->mps.previous_frame.empty()){
    current_model->mps.previous_frame = current_frame.clone();
    return;
  }

  const cv::Size subPixWinSize(10, 10);

  cv::Mat gray;
  if (current_model->mps.previous_frame.type() == CV_8UC3)
    cv::cvtColor(current_model->mps.previous_frame, gray, CV_BGR2GRAY);
  else if (current_model->mps.previous_frame.type() == CV_8UC1)
    gray = current_model->mps.previous_frame.clone();

  current_model->mps.points_test[0].clear();
  current_model->mps.points_test[1].clear();

  for (int i = 0; i < 6; ++i){
    std::vector<cv::Point2f> points_per_component;

    cv::Mat mask = component_image == i;
    if (i == 2 || i == 3 ) continue;
    //|| i > 3
    cv::goodFeaturesToTrack(gray, points_per_component, 15, 0.01, 10, mask, 3, 0, 0.04);
    if (points_per_component.empty()) continue;
    cv::cornerSubPix(gray, points_per_component, subPixWinSize, cv::Size(-1, -1), term_crit_);
    current_model->mps.points_test[0].insert(current_model->mps.points_test[0].end(), points_per_component.begin(), points_per_component.end());
  }

  if (current_model->mps.points_test[0].empty()) return;

  current_model->mps.tracked_points_.clear();
  for (size_t i = 0; i < current_model->mps.points_test[0].size(); ++i){
    cv::Vec3f &point_on_model = current_model->mps.front_intersection_image_.at<cv::Vec3f>(std::round(current_model->mps.points_test[0][i].y), std::round(current_model->mps.points_test[0][i].x));
    if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){
      current_model->mps.tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));
      current_model->mps.tracked_points_.back().point_in_view = false;
      current_model->mps.tracked_points_.back().point_tracked_on_model = false;
    }
    else{
      unsigned char idx = component_image.at<unsigned char>(std::round(current_model->mps.points_test[0][i].y), std::round(current_model->mps.points_test[0][i].x));
      auto pt_in_world_coords = current_model->GetComponentPose(idx).InverseTransformPoint(point_on_model);
      auto back_again = current_model->GetComponentPose(idx).TransformPoint(pt_in_world_coords);

      current_model->mps.tracked_points_.push_back(TrackedPoint(current_model->GetComponentPose(idx).InverseTransformPoint(point_on_model), current_model->mps.points_test[0][i]));
      current_model->mps.tracked_points_.back().component_idx = idx;
      current_model->mps.tracked_points_.back().point_tracked_on_model = true;
      current_model->mps.tracked_points_.back().point_in_view = true;

    }
  }

  current_model->mps.previous_frame = gray.clone();
  current_model->mps.is_initialised = true;
  LKTracker::TrackLocalPoints(current_frame, current_model);

}

void ArticulalatedLKTrackerFrameToFrame::InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model){

  current_model->mps.is_initialised = true;

}

void ArticulalatedLKTrackerFrameToFrame::InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &component_image){

  current_model->mps.is_initialised = true;

}


