#include <cinder/app/App.h>

#include "../../../../include/ttrack/track/localizer/features/lk_tracker.hpp"
#include "../../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

LKTracker::LKTracker(boost::shared_ptr<MonocularCamera> camera) : FeatureLocalizer(camera) {
  win_size_ = cv::Size(31, 31);
  term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
}

void LKTracker::TrackLocalPoints(cv::Mat &current_frame){

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

