#ifndef _SURGICAL_TOOL_TRACKER_HPP_
#define _SURGICAL_TOOL_TRACKER_HPP_
#include "headers.hpp"
#include "tracker.hpp"

namespace ttrk{


  class SurgicalToolTracker : public Tracker{

  public:
    
    SurgicalToolTracker(const int width, const int height):width_(width),height_(height){}

    ~SurgicalToolTracker();

    virtual bool Init();
    
  protected:

    const int width_;
    const int height_;

    virtual const cv::Mat ProjectShapeToSDF() const;


    bool FindConnectedRegions();
    void InitPoseFromMOITensor();

    

  };




}


#endif //_SURGICAL_TOOL_TRACKER_HPP_
