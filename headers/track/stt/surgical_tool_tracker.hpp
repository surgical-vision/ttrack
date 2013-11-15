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
    SurgicalToolTracker(const float radius, const float height);

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

    const float radius_; /**< The tool radius in metric units. Used for constructing new tool models. */
    const float height_; /**< The tool tool length in metric units. Used for constructing new tool models. */

    void ShiftCenter(cv::Vec2f &center_of_mass,const cv::Vec2f &central_axis, double length) const;

    /**
    * Find connected regions in the classified image. Each blob is a candidate for a surgical instrument. 
    * @param[out] connected_regions All of the connected regions found in the image, sorted in descenting order by the number of pixels that make up the region.
    * @return The success of finding the blobs. 
    */
    bool FindConnectedRegions(const cv::Mat &frame, std::vector<std::vector<cv::Vec2i> >&connected_regions);
    bool ThresholdImage(const cv::Mat &image, std::vector<std::vector<cv::Point> > &contours) const;
    void FindSingleRegionFromContour(const std::vector<cv::Point> &contour,std::vector<cv::Vec2i> &connected_region) const;
    void GetContours(const cv::Mat &image, std::vector<std::vector<cv::Point> > &contours) const;
    void CheckCentralAxisDirection(const cv::Vec2f &center_of_mass, cv::Vec2f &horizontal_axis) const;


    template<typename T>
    T GetCenter(const std::vector<T> &contour) const ;

  };

  template<typename T>
  T SurgicalToolTracker::GetCenter(const std::vector<T> &contour) const {
    cv::Vec2i center(0,0);
      for(size_t i=0;i<contour.size();i++){
    center = center + (cv::Vec2i)contour[i];
    }
    center[0] = center[0]/(int)contour.size();
    center[1] = center[1]/(int)contour.size();
    return center;
  }


}


#endif //_SURGICAL_TOOL_TRACKER_HPP_
