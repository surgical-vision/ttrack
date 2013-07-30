#ifndef __MONO_TOOL_TRACKER_HPP__
#define __MONO_TOOL_TRACKER_HPP__
#include "surgical_tool_tracker.hpp"

namespace ttrk {

  class MonocularToolTracker : public SurgicalToolTracker {

  public:
    MonocularToolTracker(const float  radius, const float  height, const std::string &calibration_filename);
    
    virtual ~MonocularToolTracker() {};

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init();
    virtual void DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const;
    
  protected:

    
    /**
    * Initialise the 2D pose of the instrument from the moments of the connected region.
    * @param[in] A connected region of pixels.
    */
    void Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region);
    
    /**
    * Finds the center of mass of a connceted region of pixels.
    * @param[in] A cluster of pixels coordindates.
    * @param[out] The cluster center.
    */
    const cv::Vec2i FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const;


    MonocularCamera camera_;

  };

  
  


}


#endif
