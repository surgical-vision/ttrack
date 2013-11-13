#ifndef __POSE_HPP__
#define __POSE_HPP__

#include "../../deps/quaternion/inc/quaternion.hpp"

namespace ttrk {

  struct Pose {

    inline Pose():rotation_(0,cv::Vec3f(0,0,0)),translation_(0,0,0){}

    inline Pose(const cv::Vec3f &v, const cv::Vec4f &r): translation_(v), rotation_(r[0],cv::Vec3f(r[1],r[2],r[3])){}

    inline Pose(const cv::Vec3f &v, const sv::Quaternion &r):translation_(v),rotation_(r){}

    Pose operator=(const cv::Mat &that);

    inline Pose(const Pose &that){
      translation_ = that.translation_;
      rotation_ = that.rotation_;
      translational_velocity_ = that.translational_velocity_;
    }

    inline cv::Vec3f Transform(const cv::Vec3f &to_transform) const {
      return rotation_.RotateVector(to_transform) + translation_;
    }

    inline cv::Vec3f InverseTransform(const cv::Vec3f &to_inv_transform) const {
      cv::Vec3f t = to_inv_transform - translation_;
      sv::Quaternion inverse_rotation = rotation_.Inverse();
      return inverse_rotation.RotateVector(t);
    }

    inline Pose Inverse() const {

      Pose p;
      p.rotation_ = rotation_.Inverse();
      p.translation_ = -p.rotation_.RotateVector(translation_);
      return p;
    
    }

    cv::Vec3f GetDOFDerivatives(const int dof, const cv::Vec3f &point) const ;

    operator cv::Mat() const;

    cv::Vec3f translation_;
    sv::Quaternion rotation_;

    /**** EXPERIMENTAL ****/
    cv::Vec3f translational_velocity_;
    //cv::Vec3f rotational_velocity;

  };

  inline std::ostream &operator<<(std::ostream &os, const Pose &p){
    os << "[" << p.rotation_ << ", " << p.translation_[0] << ", " << p.translation_[1] << ", " << p.translation_[2] << "]";
    return os;
  }

  inline Pose CombinePoses(const Pose &a, const Pose &b) {

    Pose p;
    p.translation_ = a.translation_ + a.rotation_.RotateVector(b.translation_);
    p.rotation_ = a.rotation_ * b.rotation_;
    return p;

  }


}

#endif
