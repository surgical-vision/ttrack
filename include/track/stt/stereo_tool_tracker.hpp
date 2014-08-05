#ifndef __STEREO_TOOL_TRACKER_HPP__
#define __STEREO_TOOL_TRACKER_HPP__

#include <cv.h>

#include "surgical_tool_tracker.hpp"


namespace ttrk {

  class StereoToolTracker : public SurgicalToolTracker {

  public:

    StereoToolTracker(const std::string &model_parameter_file, const std::string &calibration_filename);

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

    void ShiftToTip(const cv::Vec3d &central_axis, cv::Vec3d &center_of_mass); 
    void InitIn2D(const std::vector<cv::Vec2i> &connected_region, cv::Vec3d &center_of_mass_3d, cv::Vec3d &central_axis_3d, boost::shared_ptr<MonocularCamera> cam, KalmanTracker &tm);
    cv::Vec2d FindCenterOfMassIn2D(const std::vector<cv::Vec2i> &connected_region) const;

    virtual void SetHandleToFrame(boost::shared_ptr<sv::Frame> image);

    //void CreateDisparityImage();

    void Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, KalmanTracker &tracked_model);

    cv::Vec3d FindCenterOfMass(const cv::Mat &point_cloud) const ; 
    //cv::Vec3d FindClusterMode(const cv::Mat &point_cloud, const cv::Mat &classification_map) const ;
    cv::Vec3d FindPrincipalAxisFromMOITensor(const cv::Vec3d center_of_mass, const cv::Mat &point_cloud) const ;
   
    boost::shared_ptr<StereoCamera> camera_;

  };

}

#endif
