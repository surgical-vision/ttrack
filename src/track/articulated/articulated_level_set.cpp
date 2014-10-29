#include "../../../include/track/articulated/articulated_level_set.hpp"

using namespace ttrk;

void ArticulatedLevelSet::ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image){

  //generate each signed distance function for each component - this allows us to know which component of the articulated model
  //is 'generating' each value is the composite SDF

  //for (size_t i = 0; i < component_trackers_.size(); ++i){

  //  Model &m = mesh->


  //}


}

