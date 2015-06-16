#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__

#include <cinder/app/Renderer.h>

#include "../model/model.hpp"
#include "../../utils/image.hpp"

namespace ttrk {

  /**
  * @class Localizer
  * @brief An abstract class to represent pose localization algorithms.
  */
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

    /**
    * Test for convergence.
    * @return True for converged, false otherwise.
    */
    virtual bool HasConverged() const = 0;

    /**
    * Setter for frame count variable.
    * @param[in] fc The current frame count.
    */
    void SetFrameCount(int fc) { frame_count_ = fc; }

    /**
    * Accessor for the progress frame for visualization in the GUI.
    * @return The progress frame.
    */
    cv::Mat GetProgressFrame() { return progress_frame_; }

  protected:

    cv::Mat progress_frame_; /**< Localizer progress frame. For viewing progress in a GUI or something similar. */

    int frame_count_; /**< Current frame count */

  };


}


#endif
