#ifndef __DESCRIPTOR_HPP__
#define __DESCRIPTOR_HPP__

#include <cinder/Vector.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

#include "../../model/model.hpp"

namespace ttrk{

  class Descriptor {

  public:

    Descriptor() {};
    
    Descriptor(const cv::Vec3f pt, const cv::Vec3f normal, const cv::Mat &descriptor);
    
    cv::Mat GetDescriptor() { return descriptor_; }
    cv::Mat GetDescriptor() const { return descriptor_.clone(); }

    double GetPercentageHits() { if (total_num_attempts_ > 0) return (double)times_matched_ / total_num_attempts_; else return 0; }

    size_t GetTimesMatched() const { return times_matched_; }
    size_t GetTotalNumberOfAttempts() const { return total_num_attempts_; }

    void IncrementNumberOfMatches() { times_matched_++; }
    void IncrementTotalNumberOfAttempts() { total_num_attempts_++; }
    
    bool ShouldUseThisDescriptor(Pose &pose) const;

    ci::Vec3f ciGetPointInEyeSpace(Pose &pose) const;
    cv::Vec3f cvGetPointInEyeSpace(Pose &pose) const { auto i = ciGetPointInEyeSpace(pose); return cv::Vec3f(i[0], i[1], i[2]); }

  protected:

    ci::Vec3f coordinate_; //in model space
    ci::Vec3f normal_; //in model space
    cv::Mat descriptor_; //the actual 
    size_t times_matched_; //number of 'successful' finds
    size_t total_num_attempts_; //total number of attemps
  };


  struct MatchedPair {
    
    cv::Vec3f eye_coordinates;
    cv::Point2f image_coordinates;
    float matching_distance; 

  };


}

#endif