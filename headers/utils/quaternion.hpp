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
    inline double X() const;
    inline double Y() const;
    inline double Z() const;
    inline double W() const;

    inline friend Quaternion operator*(const Quaternion &a, const Quaternion &b);

  protected:

    boost::math::quaternion<double> internal_quaternion_;


  };

  Quaternion operator*(const Quaternion &a, const Quaternion &b) {
    
    Quaternion q;
    q.internal_quaternion_ = a.internal_quaternion_ * b.internal_quaternion_;
    return q;
  }

  double Quaternion::X() const {
    return internal_quaternion_.R_component_2();
  }
  double Quaternion::Y() const {
    return internal_quaternion_.R_component_3();
  }
  double Quaternion::Z() const {
    return internal_quaternion_.R_component_4();
  }
  double Quaternion::W() const {
    return internal_quaternion_.R_component_1();
  }
}


#endif
