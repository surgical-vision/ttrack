#ifndef __QUATERNION_HPP__
#define __QUATERNION_HPP__

#include <cv.h>
#include <boost/math/quaternion.hpp>

namespace sv {

  class Quaternion {
  
  public:
    Quaternion(){}
    Quaternion(boost::math::quaternion<double> &x):internal_quaternion_(x) {}
    Quaternion(const double angle, const cv::Vec3f &axis);
    static Quaternion FromVectorToVector(const cv::Vec3f &from, const cv::Vec3f to);
    cv::Vec3f RotateVector(const cv::Vec3f &to_rotate) const ;
    Quaternion Normalize() const ;

  protected:

    boost::math::quaternion<double> internal_quaternion_;


  };
}


#endif
