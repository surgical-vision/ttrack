#ifndef __MONO_PWP3D_HPP__
#define __MONO_PWD3D_HPP__

#include "../localizer.hpp"

namespace ttrk {

  class MonoPWP3D : public Localizer {

  public: 
    virtual cv::Mat TrackTargetInFrame(std::vector<KalmanTracker>::iterator model){


    }

  };

  class StereoPWP3D : public Localizer {
    
  public: 
    virtual cv::Mat TrackTargetInFrame(std::vector<KalmanTracker>::iterator model){


    }



  };


}


#endif
