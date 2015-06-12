#ifndef __STEREO_TOOL_TRACKER_HPP__
#define __STEREO_TOOL_TRACKER_HPP__

#include <cv.h>

#include "surgical_tool_tracker.hpp"

namespace ttrk {

  class StereoToolTracker : public SurgicalToolTracker {

  public:
    
    /**
    * Construct a trackable tool specifying the width and height of the tool in metric units and a calibration filename for the cameras.
    * @param[in] model_parameter_file The url of the file containing the model to be tracked.
    * @param[in] calibration_filename The url of the file containing the camera calibration.
    * @param[in] results_dir The directory where to save the results from tracked models.
    * @param[in] localizer_type The type of localizer to use in the frame-by-frame pose estimation.
    * @param[in] number_of_labels The number of labels used in detection. This includes the background label.
    */
    StereoToolTracker(const std::string &model_parameter_file, const std::string &calibration_filename, const std::string &results_dir, const LocalizerType &localizer_type, const size_t number_of_labels);

    /**
    * Destructor
    */
    virtual ~StereoToolTracker(){}

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init();

    /**
    * Helper to cast the inherited image data to a sv::StereoFrame object.
    * @return The frame cast to the stereo type.
    */
    boost::shared_ptr<sv::StereoFrame> StereoFrame() { return boost::dynamic_pointer_cast<sv::StereoFrame,sv::Frame>(frame_); }
    
  protected:

    virtual void SetHandleToFrame(boost::shared_ptr<sv::Frame> image);

    void ShiftToTip(const cv::Vec3d &central_axis, cv::Vec3d &center_of_mass); 

    void InitIn2D(const std::vector<cv::Vec2i> &connected_region, cv::Vec3d &center_of_mass_3d, cv::Vec3d &central_axis_3d, boost::shared_ptr<MonocularCamera> cam, boost::shared_ptr<Model> tm);

    cv::Vec2d FindCenterOfMassIn2D(const std::vector<cv::Vec2i> &connected_region) const;



    void Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, boost::shared_ptr<Model> tracked_model);

    cv::Vec3d FindCenterOfMass(const cv::Mat &point_cloud) const ; 

    cv::Vec3d FindPrincipalAxisFromMOITensor(const cv::Vec3d center_of_mass, const cv::Mat &point_cloud) const ;
   
    boost::shared_ptr<StereoCamera> camera_;

  };

}

#endif
