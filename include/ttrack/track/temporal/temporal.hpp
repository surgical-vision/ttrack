#ifndef __TEMPORAL_HPP__
#define __TEMPORAL_HPP__

#include <boost/shared_ptr.hpp>

#include "../model/model.hpp"

namespace ttrk {

  /**
  * @class TemporalTracker
  * @brief An abstract class to represent temporal tracking models.
  * Derived from this class to enable temporal tracking for your models. The tracker maintains a struct TemporalTrackedModel which combines a TemporalTracker with a Model.
  * After each pose localization the model's new pose estimate is incorporated to the temporal tracker and an new estimate is made.
  */

  class TemporalTracker {

  public:

    virtual void UpdatePoseWithMotionModel(boost::shared_ptr<Model> model) = 0;

    virtual void Init(std::vector<float> &start_pose) = 0;

  protected:

  };

  class KalmanFilterTracker : public TemporalTracker {

  public:

    virtual void UpdatePoseWithMotionModel(boost::shared_ptr<Model> model);
    
    virtual void Init(std::vector<float> &start_pose);

  protected:

    cv::KalmanFilter filter_; /**< The Kalman Filter used to track the class. */

  };


}


#endif