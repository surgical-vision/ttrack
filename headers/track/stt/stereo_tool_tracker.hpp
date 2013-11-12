#ifndef __STEREO_TOOL_TRACKER_HPP__
#define __STEREO_TOOL_TRACKER_HPP__
#include "surgical_tool_tracker.hpp"
#include <cv.h>

namespace ttrk {

  class StereoToolTracker : public SurgicalToolTracker {

  public:

    StereoToolTracker(const float  radius, const float  height, const std::string &config_dir, const std::string &calibration_filename);

    virtual ~StereoToolTracker(){}

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init();

    boost::shared_ptr<sv::StereoFrame> StereoFrame() { return boost::dynamic_pointer_cast<sv::StereoFrame,sv::Frame>(frame_); }
    
    virtual void DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const;
  
  protected:

    void ShiftToTip(const cv::Vec3f &central_axis, cv::Vec3f &center_of_mass); 
    void InitIn2D(const std::vector<cv::Vec2i> &connected_region, cv::Vec3f &center_of_mass_3d, cv::Vec3f &central_axis_3d, boost::shared_ptr<MonocularCamera> cam);
    const cv::Vec2i FindCenterOfMassIn2D(const std::vector<cv::Vec2i> &connected_region) const;

    virtual void ProcessFrame();

    virtual void SetHandleToFrame(boost::shared_ptr<sv::Frame> image);

    //void CreateDisparityImage();

    void Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, KalmanTracker &tracked_model);

    cv::Vec3f FindCenterOfMass(const cv::Mat &point_cloud) const ; 
    //cv::Vec3f FindClusterMode(const cv::Mat &point_cloud, const cv::Mat &classification_map) const ;
    cv::Vec3f FindPrincipalAxisFromMOITensor(const cv::Vec3f center_of_mass, const cv::Mat &point_cloud) const ;
   
    boost::shared_ptr<StereoCamera> camera_;

  };

}

#endif
