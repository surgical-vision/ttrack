#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"

namespace ttrk {

  class ArticulatedLevelSet : public Localizer {

  public:

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  protected:

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);

    std::vector<StereoPWP3D> component_trackers_; /**< A selection of trackers each of which compute the level sets for each articulated component. */
    
  };





}


#endif
