#ifndef __MONO_PWP3D_HPP__
#define __MONO_PWD3D_HPP__

#include "pwp3d.hpp"
#include "../localizer.hpp"

namespace ttrk {

  class MonoPWP3D : public PWP3D {

  public: 

    MonoPWP3D(const std::string &config_dir,boost::shared_ptr<MonocularCamera> camera) : PWP3D(config_dir,camera) {}

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);

  };


}


#endif
