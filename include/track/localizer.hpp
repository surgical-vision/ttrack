#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__

#include <cinder/app/Renderer.h>

#include "track/model/model.hpp"
#include "../../deps/image/image/image.hpp"



namespace ttrk {

  class Localizer {

  public:

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) = 0;
    
  };


}


#endif
