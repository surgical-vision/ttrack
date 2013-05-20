#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__
#include "kalman.hpp"

namespace ttrk {

  class Localizer {

  public:

    virtual cv::Mat TrackTargetInFrame(KalmanTracker &model) = 0;

  };


}


#endif
