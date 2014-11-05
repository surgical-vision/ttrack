#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"
#include "../model/node.hpp"

namespace ttrk {

  class ArticulatedLevelSet : public Localizer {

  public:

    ArticulatedLevelSet(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  protected:

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);

    std::vector<StereoPWP3D> component_trackers_; /**< A selection of trackers each of which compute the level sets for each articulated component. */
    
    ci::gl::GlslProg front_depth_;  /**< Shader to compute the front depth buffer. */
    ci::gl::GlslProg back_depth_and_contour_;  /**< Shader to compute the back depth buffer and contour. */

    struct ComponentData {

      cv::Mat sdf_image_;
      cv::Mat front_intersection_image_;
      cv::Mat back_intersection_image_;
      
      cv::Mat sdf_;

      Node::Ptr model_;

      //body (roll) -> head (w.p) -> no-geom (w.y) -> no-geom (clasper-base) -> 

      //component data should mirror structure of Model-tree

      //SDF for this component

      //index of the component in the model (should this be a Model(Node) thing)

      //jacobian could

      //detection color/label mapping

    };

    boost::shared_ptr<StereoCamera> stereo_camera_;

  };





}


#endif
