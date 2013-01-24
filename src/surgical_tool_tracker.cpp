#include "../headers/surgical_tool_tracker.hpp"

using namespace ttrk;

bool SurgicalToolTracker::Init(){

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(connected_regions)) return false;

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){
        
    Init2DPoseFromMOITensor(*connected_region);
  
  }

  return true;

}

bool SurgicalToolTracker::FindConnectedRegions(std::vector<std::vector<cv::Vec2i> >&connected_regions){

  //iterate over frame_

  //opencv flood

  return false;
}

void SurgicalToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region){

  const cv::Vec2i center_of_mass = FindCenterOfMass(connected_region);

}

const cv::Vec2i SurgicalToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2i ret(0,0);
  return ret;

}

const cv::Mat SurgicalToolTracker::ProjectShapeToSDF() const {

  cv::Mat sdf_image(frame_->size(),CV_32FC1);

  return sdf_image;


}
