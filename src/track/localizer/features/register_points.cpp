#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <numeric>

#include "../../../include/ttrack/track/localizer/features/register_points.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/pwp3d.hpp"
#include "../../../include/ttrack/constants.hpp"

using namespace ttrk;

PointRegistration::PointRegistration(boost::shared_ptr<MonocularCamera> camera) : FeatureLocalizer(camera) {  }


void PointRegistration::TrackLocalPoints(cv::Mat &current_frame){

  cv::cvtColor(current_frame, current_frame_gray_, CV_BGR2GRAY);

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  cv::SurfFeatureDetector detector(minHessian);

  std::vector<cv::KeyPoint> keypoints;

  detector.detect(current_frame_gray_, keypoints, CreateMask());

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;

  cv::Mat descriptors;

  extractor.compute(current_frame_gray_, keypoints, descriptors);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match(descriptors, descriptors_previous_frame_, matches);

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  for (int i = 0; i < descriptors.rows; i++)
  {
    if (matches[i].distance <= std::max(2 * min_dist, 0.02))
    {
      int midx = matches[i].queryIdx;
      tracked_points_[i].found_image_point = keypoints[midx].pt;
    }
  }  

}

void PointRegistration::InitializeTracker(cv::Mat &current_frame, const Pose &pose){

  cv::Mat gray;
  cv::cvtColor(current_frame, gray, CV_BGR2GRAY);

  tracked_points_.clear();

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  cv::SurfFeatureDetector detector(minHessian);

  std::vector<cv::KeyPoint> keypoints;

  detector.detect(gray, keypoints);

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;
  
  extractor.compute(gray, keypoints, descriptors_previous_frame_);

  for (size_t i = 0; i < keypoints.size(); ++i){

    cv::Vec3f &point_on_model = front_intersection_image_.at<cv::Vec3f>(keypoints[i].pt.y, keypoints[i].pt.x);

    if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){

      tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));

    }
    else{

      tracked_points_.push_back(TrackedPoint(pose.InverseTransformPoint(point_on_model), keypoints[i].pt));

    }
  }

  //we recompute the points each time

  previous_frame_gray_ = gray.clone();


}