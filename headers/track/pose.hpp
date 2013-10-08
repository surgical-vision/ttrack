#ifndef __POSE_HPP__
#define __POSE_HPP__

#include "../../deps/quaternion/inc/quaternion.hpp"

namespace ttrk {

  struct Pose {

    inline Pose():rotation_(0,cv::Vec3f(0,0,0)),translation_(0,0,0){}

    inline Pose(const cv::Vec3f &v, const cv::Vec4f &r): translation_(v), rotation_(r[0],cv::Vec3f(r[1],r[2],r[3])){}

    inline Pose(const cv::Vec3f &v, const sv::Quaternion &r):translation_(v),rotation_(r){}

    inline Pose operator=(const cv::Mat &that);

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

    inline operator cv::Mat() const;

    cv::Vec3f translation_;
    sv::Quaternion rotation_;

    /**** EXPERIMENTAL ****/
    cv::Vec3f translational_velocity_;
    //cv::Vec3f rotational_velocity;

  };

  inline Pose CombinePoses(const Pose &a, const Pose &b) {

    Pose p;
    p.translation_ = a.translation_ + a.rotation_.RotateVector(b.translation_);
    p.rotation_ = a.rotation_ * b.rotation_;
    return p;

  }

  Pose Pose::operator=(const cv::Mat &that){
    if(that.size() == cv::Size(1,9)){

      const cv::Vec3f translation(that(cv::Range(0,3),cv::Range::all()));     
      const cv::Vec3f velocity(that(cv::Range(3,6),cv::Range::all()));
      const cv::Vec3f rotation(that(cv::Range(6,9),cv::Range::all()));
      this->rotation_ = sv::Quaternion(rotation);
      this->translation_ = translation;
      this->translational_velocity_ = velocity;

    }else if(that.size() == cv::Size(9,1)){

      const cv::Vec3f translation(that(cv::Range::all(),cv::Range(0,3)));
      const cv::Vec3f velocity(that(cv::Range::all(),cv::Range(3,6)));
      const cv::Vec3f rotation(that(cv::Range::all(),cv::Range(6,9)));
      this->rotation_ = sv::Quaternion(rotation);
      this->translation_ = translation;
      this->translational_velocity_ = velocity;

    }else{
      throw(std::runtime_error("Error, invalid assignement to ttrk::Pose from cv::Mat. Dimensions do not match!\n"));
    }
    return *this;
  }


  Pose::operator cv::Mat() const {
    cv::Mat r(9,1,CV_32FC1);
    cv::Vec3f euler = rotation_.EulerAngles();    
    for(int i=0;i<3;i++){
      r.at<float>(i,0) = translation_[i];
      r.at<float>(i+3,0) = translational_velocity_[i];      
      r.at<float>(i+6,0) = euler[i];
    }
    return r;  
  }

}

#endif
