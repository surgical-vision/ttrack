#ifndef __LK_TRACKER__
#define __LK_TRACKER__

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../../headers.hpp"
#include "../../model/pose.hpp"
#include "../../../utils/camera.hpp"
#include "../../model/model.hpp"
#include "../../../constants.hpp"
#include "feature_localizer.hpp"

namespace ttrk{

  class LKTracker : public FeatureLocalizer {

  public:

    LKTracker(boost::shared_ptr<MonocularCamera> camera);

    virtual void TrackLocalPoints(cv::Mat &current_frame);

    virtual void InitializeTracker(cv::Mat &current_frame, const Pose &pose);
    
  protected:

    std::vector<TrackedPoint> tracked_points_;

    std::vector<cv::Point2f> points_test[2];

    cv::Mat previous_frame_gray_;
    cv::Mat current_frame_gray_;
    
    cv::Mat front_intersection_image_; //for masking

    cv::Size win_size_;
    cv::TermCriteria term_crit_;

    boost::shared_ptr<MonocularCamera> camera_;
    
    float current_error_;

  };



}

#endif