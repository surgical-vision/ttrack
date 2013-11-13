#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {
  
  class StereoPWP3D : public PWP3D {

  public: 

    StereoPWP3D(const std::string &config_dir, boost::shared_ptr<StereoCamera> camera) : PWP3D(config_dir,camera->rectified_left_eye()) , stereo_camera_(camera) {}

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);
    
    boost::shared_ptr<StereoCamera> &GetStereoCamera() { return stereo_camera_; } //* Probably this shouldn't be a reference. Change it so it is not. */

  protected:   

    void DrawModelOnBothFrames(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas);

    //bool HasGradientDescentConverged(std::vector<Pose> &convergence_test_values, Pose &current_estimate) const;
    bool HasGradientDescentConverged(std::deque<Pose> &previous_poses, const Pose &pose) const ;
    bool HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const ;
    bool HasGradientDescentConverged__new(std::vector<cv::Mat> &convergence_test_values, cv::Mat &current_estimate) const;
    
    cv::Vec3f GetDOFDerivativesRightEye(const int dof, const Pose &pose, const cv::Vec3f &point_) ;
    cv::Mat GetPoseDerivativesRightEye(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image);
    
    bool SetupEye(const int eye, Pose &pose);

    boost::shared_ptr<StereoCamera> stereo_camera_;
    //cv::Mat ROI_left_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    //cv::Mat ROI_right_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };


}

#endif
