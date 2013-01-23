#ifndef _SURGICAL_TOOL_TRACKER_HPP_
#define _SURGICAL_TOOL_TRACKER_HPP_
#include "headers.hpp"
#include "tracker.hpp"

namespace ttrk{


  class SurgicalToolTracker : public Tracker{

  public:
    
    SurgicalToolTracker(const int width, const int height):width_(width),height_(height){}

    ~SurgicalToolTracker(){};

    virtual bool Init();
    
  protected:

    const int width_;
    const int height_;

    virtual const cv::Mat ProjectShapeToSDF() const;


    bool FindConnectedRegions(std::vector<std::vector<cv::Vec2i> >&connected_regions);
    void Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region);
    const cv::Vec2i SurgicalToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const;
    

  };




}


#endif //_SURGICAL_TOOL_TRACKER_HPP_
