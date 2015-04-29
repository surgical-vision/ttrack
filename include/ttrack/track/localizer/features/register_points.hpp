#ifndef __REGISTER_POINTS_HPP__
#define __REGISTER_POINTS_HPP__

#include "../../../headers.hpp"
#include "../../model/pose.hpp"
#include "../../../utils/camera.hpp"
#include "../../../track/model/model.hpp"
#include "feature_localizer.hpp"

namespace ttrk {

  const int NUM_DESCRIPTOR = 120;
  const int MATCHING_DISTANCE_THRESHOLD = 10;
  const double DESCRIPTOR_SIMILARITY_THRESHOLD = 200.0;

  class PointRegistration : public FeatureLocalizer {

  public:

    PointRegistration(boost::shared_ptr<MonocularCamera> camera);

    virtual void TrackLocalPoints(cv::Mat &current_frame);

    virtual void InitializeTracker(cv::Mat &current_frame, const Pose &pose);

 
  protected:

    cv::Mat descriptors_previous_frame_;

  };


}

#endif
