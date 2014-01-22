#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {

  enum CameraEye { LEFT = 0, RIGHT = 1 , INV = 2};
  inline CameraEye operator++(CameraEye &e, int){
    const CameraEye eye = e;
    const int i = static_cast<int>(e);
    e = static_cast<CameraEye>(i+1);
    return eye;
  }
  
  class StereoPWP3D : public PWP3D {

  public: 

    StereoPWP3D(boost::shared_ptr<StereoCamera> camera) : PWP3D(camera->rectified_left_eye()) , stereo_camera_(camera) {}

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);
    
  protected:   

    virtual void GetFastDOFDerivs(const Pose &pose, double *pose_derivs, double *intersection);
    void GetFastDOFDerivsLeft(const Pose &pose, double *pose_derivs, double *intersection);
    void GetFastDOFDerivsRight(const Pose &pose, double *pose_derivs, double *intersection);
    
    bool SwapEye(Pose &pose);
    void SwapToRight(Pose &pose);
    void SwapToLeft(Pose &pose);

    CameraEye current_eye_;
    boost::shared_ptr<StereoCamera> stereo_camera_;

  };


}

#endif
