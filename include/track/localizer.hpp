#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__

#include <cinder/app/Renderer.h>

#include "track/model/model.hpp"
#include "../../deps/image/image/image.hpp"



namespace ttrk {

  class Localizer {

  public:

    /**
    * Do single frame pose estimation. This method receives a model (which may or may not have some initial estimate of pose) and tries to 
    * estimate a new estimate for the object in the new frame. The update is applied to the model in this method.
    * @param[in] model The model to align to the image data.
    * @param[in] frame The current frame to perform alignment against.
    */
    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) = 0;
    
    /**
    * Virtual destructor.
    */
    virtual ~Localizer() {}

    virtual bool NeedsNewFrame() const = 0;

    virtual void UpdateErrorAccumulator() = 0;


  };


}


#endif
