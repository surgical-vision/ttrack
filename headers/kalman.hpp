#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

#include <cv.h>
#include <boost/shared_ptr.hpp>
#include "model.hpp"

namespace ttrk {

 /**
  * @struct KalmanTracker
  * @brief A container to hold a tracked model and the kalman filter associated with it.
  */
  struct KalmanTracker {

    cv::KalmanFilter filter_; /**< The Kalman Filter used to track the class. */
    boost::shared_ptr<Model> model_; /**< The tracked model. */
    cv::Mat temporary_update_pose; /**< A store for the pose update. May be removed. */
    
  };



}

#endif
