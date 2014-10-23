#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"

namespace ttrk {

  class ArticulatedLevelSet : public Localizer {

  public:

    virtual Pose TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  protected:

    std::vector<StereoPWP3D> component_trackers_; /**< A selection of trackers each of which compute the level sets for each articulated component. */
    
  };





}


#endif
