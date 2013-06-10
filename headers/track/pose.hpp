#ifndef __POSE_HPP__
#define __POSE_HPP__

#include "../utils/quaternion.hpp"

namespace ttrk {

  struct Pose {

    inline Pose():rotation_(0,cv::Vec3f(0,0,0)),translation_(0,0,0){}
    
    inline Pose(const Pose &that){
      translation_ = that.translation_;
      rotation_ = that.rotation_;
    }

    inline cv::Vec3f Transform(const cv::Vec3f &to_transform) const {
      return rotation_.RotateVector(to_transform) + translation_;
    }

    cv::Vec3f translation_;
    sv::Quaternion rotation_;

  };


}

#endif
