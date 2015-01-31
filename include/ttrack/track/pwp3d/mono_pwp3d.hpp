#ifndef __MONO_PWP3D_HPP__
#define __MONO_PWD3D_HPP__

#include "pwp3d.hpp"
#include "../localizer.hpp"

namespace ttrk {

  // WARNING - THIS CLASS HAS NOT BEEN TESTED VERY WELL AS I DON'T REALLY WORK WITH MONO CAMERAS. sorry :(

  /**
  * @class MonoPWP3D
  * @brief A class to do PWP3D type tracking with monocular cameras.
  */
  class MonoPWP3D : public PWP3D {

  public: 

    /**
    * Default constructor.
    * @param[in] camera The camera we are using for projection etc.
    */
    MonoPWP3D(boost::shared_ptr<MonocularCamera> camera);

    /**
    * Specialization of actual frame pose locationization for monocular.
    * @param[in] model The model we are tracking. Pose is updated inside this loop.
    */
    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  protected:

    boost::shared_ptr<MonocularCamera> camera_; /**< Representation of the camera. */

  };


}


#endif
