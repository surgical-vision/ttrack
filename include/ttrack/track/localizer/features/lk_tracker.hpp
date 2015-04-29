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

  class LKTracker {

  public:

    LKTracker(boost::shared_ptr<MonocularCamera> camera);

    void TrackLocalPoints(cv::Mat &current_frame);

    std::vector<float> GetDerivativesForPoints(const Pose &pose);

    void SetFrontIntersectionImage(cv::Mat &im) { front_intersection_image_ = im.clone(); }

    void UpdatePointsOnModelAfterDerivatives(const Pose &pose);

    void InitializeTracker(cv::Mat &current_frame, const Pose &pose);
    
  protected:

    void UpdatePointsOnModelBeforeDerivatives(const Pose &pose);
    
    std::vector<float> GetPointDerivative(const cv::Vec3f &world_previous_, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose);

    cv::Mat CreateMask();

    std::vector<cv::Point2f> GetPointsOnPreviousImage();

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