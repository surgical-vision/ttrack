#ifndef _SURGICAL_TOOL_TRACKER_HPP_
#define _SURGICAL_TOOL_TRACKER_HPP_
#include "../../headers.hpp"
#include "../tracker.hpp"
#include "../../utils/camera.hpp"

namespace ttrk{

  /**
  * @class SurgicalToolTracker
  * @brief An implementation of a tracking system to track MIS instruments.
  */

  class SurgicalToolTracker : public Tracker{

  public:
    
    /**
    * Construct a trackable tool specifying the width and height of the tool in metric units and a calibration filename for the cameras.
    * @param[in] radius The tool radius.
    * @param[in] height The tool height.
    * @param[in] calibration_filename The url of the camera calibration file.
    */
    SurgicalToolTracker(const int radius, const int height);

    /**
    * The destructor.
    */
    virtual ~SurgicalToolTracker(){};

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init() = 0;
    
  protected:

    const int radius_; /**< The tool radius in metric units. Used for constructing new tool models. */
    const int height_; /**< The tool tool length in metric units. Used for constructing new tool models. */

    /**
    * Find connected regions in the classified image. Each blob is a candidate for a surgical instrument. 
    * @param[out] connected_regions All of the connected regions found in the image, sorted in descenting order by the number of pixels that make up the region.
    * @return The success of finding the blobs. 
    */
    bool FindConnectedRegions(std::vector<std::vector<cv::Vec2i> >&connected_regions);
    

    cv::Point GetCenter(std::vector<cv::Point> &contour) const ;

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

  };




}


#endif //_SURGICAL_TOOL_TRACKER_HPP_
