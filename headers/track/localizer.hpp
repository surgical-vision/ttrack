#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__
#include "kalman.hpp"
#include "pose.hpp"

namespace ttrk {

  class Localizer {

  public:

    virtual Pose TrackTargetInFrame(KalmanTracker model) = 0;

  };


}


#endif
