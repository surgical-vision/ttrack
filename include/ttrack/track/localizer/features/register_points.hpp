#ifndef __REGISTER_POINTS_HPP__
#define __REGISTER_POINTS_HPP__

#include "../../../headers.hpp"
#include "../../model/pose.hpp"
#include "../../../utils/camera.hpp"
#include "../../../track/model/model.hpp"
#include "feature_localizer.hpp"

namespace ttrk {

  class PointRegistration : public FeatureLocalizer {

  public:

    PointRegistration(boost::shared_ptr<MonocularCamera> camera);

    virtual void TrackLocalPoints(cv::Mat &current_frame);

    virtual void InitializeTracker(cv::Mat &current_frame, const Pose &pose);

    void UpdatePose(const Pose &pose) {
      pose_ = pose;
    }
 
  protected:

    void ComputeRootSiftFeatureFromSiftFeatures(cv::Mat &sift_descriptors) const;

    Pose pose_;

  };


}

#endif
