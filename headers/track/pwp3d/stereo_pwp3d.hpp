#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {
  
  class StereoPWP3D : public PWP3D {

  public: 

    StereoPWP3D(boost::shared_ptr<StereoCamera> camera) : PWP3D(camera->rectified_left_eye()) , stereo_camera_(camera) {}

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);
    
    boost::shared_ptr<StereoCamera> &GetStereoCamera() { return stereo_camera_; } //* Probably this shouldn't be a reference. Change it so it is not. */

  protected:   

    void DrawModelOnBothFrames(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas);

    
    cv::Vec3f GetDOFDerivativesRightEye(const int dof, const Pose &pose, const cv::Vec3f &point_) ;
    cv::Mat GetPoseDerivativesRightEye(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image);
    
    bool SetupEye(const int eye, Pose &pose);

    boost::shared_ptr<StereoCamera> stereo_camera_;

  };


}

#endif
