#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {

  class StereoPWP3D : public PWP3D {
    
  public: 
    
    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);
    boost::shared_ptr<StereoCamera> &GetStereoCamera() { return stereo_camera_; }

  protected:   

    virtual cv::Mat &ROI() { return ROI_left_; }

    cv::Mat GetRegularizedDepth(const int r, const int c, const KalmanTracker &kalman_tracker) const;

    
    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull);

    void DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const;
    Pose ApplyPointBasedRegistration(boost::shared_ptr<sv::Frame> frame, KalmanTracker &current_model );
    void ComputeDescriptorsForPointTracking(boost::shared_ptr<sv::Frame> frame, KalmanTracker current_model );

    boost::shared_ptr<StereoCamera> stereo_camera_;
    cv::Mat ROI_left_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    cv::Mat ROI_right_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };

  
  



}

#endif
