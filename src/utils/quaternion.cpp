#include "../../include/utils/quaternion.hpp"

using namespace sv;

Quaternion::Quaternion(const double angle, const cv::Vec3f &axis):internal_quaternion_(angle,axis[0],axis[1],axis[2]){}

Quaternion Quaternion::FromVectorToVector(const cv::Vec3f &from, const cv::Vec3f to){
  
  cv::Vec3f from_n,to_n;
  cv::normalize(from,from_n);
  cv::normalize(to,to_n);
      
  float d = from_n.dot(to_n);

  if(d >= 1.0){
    return boost::math::quaternion<double>();
  }

  //check if d \approx = 0

  float s = (float)sqrt( (1+d)*2 );
  float inv_s = 1.0f/s;

  cv::Vec3f axis = from_n.cross(to_n);

  Quaternion q( s*0.5f, cv::Vec3f(axis[0]*inv_s, axis[1]*inv_s, axis[2]*inv_s ));

  return q.Normalize();

}

cv::Vec3f Quaternion::RotateVector(const cv::Vec3f &to_rotate) const {
   
  const boost::math::quaternion<double> vec_quat(0,to_rotate[0],to_rotate[1],to_rotate[2]);

  boost::math::quaternion<double> rotated = (internal_quaternion_ * vec_quat) * boost::math::conj<double>(internal_quaternion_);

  return cv::Vec3f((float)rotated.R_component_2(),(float)rotated.R_component_3(),(float)rotated.R_component_3());

}
