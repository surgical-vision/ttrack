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

void PointRegistration::ComputeRootSiftFeatureFromSiftFeatures(cv::Mat &sift_descriptors) const{

  assert(sift_descriptors.type() == CV_32FC1);

  for (int r = 0; r < sift_descriptors.rows; ++r){

    //L1 normalize
    float sum = 0.0f;
    for (int c = 0; c < sift_descriptors.cols; ++c) sum += std::abs(sift_descriptors.at<float>(r, c));
    for (int c = 0; c < sift_descriptors.cols; ++c) sift_descriptors.at<float>(r, c) /= sum;

    //square root each element
    for (int c = 0; c < sift_descriptors.cols; ++c) sift_descriptors.at<float>(r, c) = std::sqrt(sift_descriptors.at<float>(r, c));

    //L2 normalize 
    sum = 0.0f;
    for (int c = 0; c < sift_descriptors.cols; ++c) sum += (sift_descriptors.at<float>(r, c)*sift_descriptors.at<float>(r, c));
    for (int c = 0; c < sift_descriptors.cols; ++c) sift_descriptors.at<float>(r, c) /= std::sqrt(sum);

  }

}


void PointRegistration::TrackLocalPoints(cv::Mat &current_frame){

  cv::cvtColor(current_frame, current_frame_gray_, CV_BGR2GRAY);

  //-- Step 1: Detect the keypoints using SIFT Detector
  

  cv::SiftDescriptorExtractor detector;

  std::vector<cv::KeyPoint> keypoints_in_current_frame;
  std::vector<cv::KeyPoint> keypoints_in_previous_frame;

  detector.detect(current_frame_gray_, keypoints_in_current_frame, CreateMask());
  detector.detect(previous_frame_gray_, keypoints_in_previous_frame, CreateMask());

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SiftDescriptorExtractor extractor;

  cv::Mat descriptors_in_current_frame;
  cv::Mat descriptors_in_previous_frame;

  extractor.compute(current_frame_gray_, keypoints_in_current_frame, descriptors_in_current_frame);
  extractor.compute(previous_frame_gray_, keypoints_in_previous_frame, descriptors_in_previous_frame);

  /**
  * Implementing RootSift from Three things everyone should know to improve object retrieval, Arandjelovic & Zisserman, CVPR 2012
  */
  
  ComputeRootSiftFeatureFromSiftFeatures(descriptors_in_current_frame);
  ComputeRootSiftFeatureFromSiftFeatures(descriptors_in_previous_frame);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector< cv::DMatch > matches;
  matcher.match(descriptors_in_current_frame, descriptors_in_previous_frame, matches);

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors_in_current_frame.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  tracked_points_.clear();

  cv::Mat test = current_frame.clone();

  for (int i = 0; i < descriptors_in_current_frame.rows; i++)
  {
    int c_idx = matches[i].queryIdx;
    int p_idx = matches[i].trainIdx;

    if (matches[i].distance <= std::max(2 * min_dist, 0.02) && l2_distance(cv::Vec2f(keypoints_in_current_frame[c_idx].pt), cv::Vec2f(keypoints_in_previous_frame[p_idx].pt)) < 20)
    {
    
      tracked_points_.push_back(TrackedPoint(pose_.InverseTransformPoint(front_intersection_image_.at<cv::Vec3f>(keypoints_in_previous_frame[p_idx].pt)), keypoints_in_previous_frame[p_idx].pt));
      tracked_points_.back().found_image_point = keypoints_in_current_frame[c_idx].pt;

      
      cv::circle(test, keypoints_in_previous_frame[p_idx].pt, 2, cv::Scalar(255, 0, 0));
      cv::circle(test, keypoints_in_current_frame[c_idx].pt, 2, cv::Scalar(0, 255, 255));

    }
  }

  //static size_t FRAMENUM = 0;
  //std::stringstream ss; ss << "z:/dump/frames" << FRAMENUM << ".jpg";
  //cv::imwrite(ss.str(), test);
  //FRAMENUM++;
  previous_frame_gray_ = current_frame_gray_.clone();


}

void PointRegistration::InitializeTracker(cv::Mat &current_frame, const Pose &pose){

  cv::Mat gray;
  cv::cvtColor(current_frame, gray, CV_BGR2GRAY);

  tracked_points_.clear();

  pose_ = pose;

  ////-- Step 1: Detect the keypoints using SURF Detector
  //int minHessian = 400;

  //cv::SurfFeatureDetector detector(minHessian);

  //std::vector<cv::KeyPoint> keypoints;

  //detector.detect(gray, keypoints, CreateMask());

  ////-- Step 2: Calculate descriptors (feature vectors)
  //cv::SurfDescriptorExtractor extractor;
  //
  //extractor.compute(gray, keypoints, descriptors_previous_frame_);

  //for (size_t i = 0; i < keypoints.size(); ++i){

  //  cv::Vec3f &point_on_model = front_intersection_image_.at<cv::Vec3f>(keypoints[i].pt.y, keypoints[i].pt.x);

  //  if (point_on_model[0] == GL_FAR || point_on_model[1] == GL_FAR || point_on_model[2] == GL_FAR){

  //    tracked_points_.push_back(TrackedPoint(cv::Vec3f(0, 0, 0), cv::Vec2f(-1, -1)));

  //  }
  //  else{

  //    tracked_points_.push_back(TrackedPoint(pose.InverseTransformPoint(point_on_model), keypoints[i].pt));

  //  }
  //}

  //we recompute the points each time

  previous_frame_gray_ = gray.clone();

}
