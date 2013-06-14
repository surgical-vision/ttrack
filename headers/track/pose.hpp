#ifndef __POSE_HPP__
#define __POSE_HPP__

#include "../utils/quaternion.hpp"

namespace ttrk {

  struct Pose {

    inline Pose():rotation_(0,cv::Vec3f(0,0,0)),translation_(0,0,0){}
    
    inline Pose(const cv::Vec3f &v, const cv::Vec4f &r): translation_(v), rotation_(r[0],cv::Vec3f(r[1],r[2],r[3])){}

    inline Pose(const cv::Vec3f &v, const sv::Quaternion &r):translation_(v),rotation_(r){}

    inline Pose operator=(const cv::Mat &that);

    inline Pose(const Pose &that){
      translation_ = that.translation_;
      rotation_ = that.rotation_;
    }

    inline cv::Vec3f Transform(const cv::Vec3f &to_transform) const {
      return rotation_.RotateVector(to_transform) + translation_;
    }

    inline operator cv::Mat() const;

    cv::Vec3f translation_;
    sv::Quaternion rotation_;

  };

   Pose Pose::operator=(const cv::Mat &that){
      if(that.size() == cv::Size(1,6)){
        const cv::Vec3f translation(that(cv::Range(0,3),cv::Range::all()));
        const cv::Vec3f rotation(that(cv::Range(3,6),cv::Range::all()));
        return Pose(translation,sv::Quaternion(rotation));
      }else if(that.size() == cv::Size(6,1)){
        const cv::Vec3f translation(that(cv::Range::all(),cv::Range(0,3)));
        const cv::Vec3f rotation(that(cv::Range::all(),cv::Range(3,7)));
        return Pose(translation,sv::Quaternion(rotation));
      }else{
        throw(std::runtime_error("Error, invalid assignement to ttrk::Pose from cv::Mat. Dimensions do not match!\n"));
      }
    }


  Pose::operator cv::Mat() const {
    cv::Mat r(6,1,CV_32FC1);
    for(int i=0;i<3;i++) r.at<float>(i,0) = translation_[i];
    cv::Vec3f euler = rotation_.EulerAngles();
    r.at<float>(3,0) = euler[0];
    r.at<float>(4,0) = euler[1];
    r.at<float>(5,0) = euler[2];
    return r;  
  }

}

#endif
