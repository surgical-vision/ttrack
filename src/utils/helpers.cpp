#include "../../include/ttrack/headers.hpp"
#include "../../include/ttrack/utils/helpers.hpp"
#include <boost/filesystem.hpp>

cv::Mat &ttrk::ConvertMatSingleToTriple(cv::Mat &im){

  //must return im!

  if(im.type() == CV_8UC3) return im;

  if(im.type() != CV_8UC1) throw(std::runtime_error("Error, unknown mask format. Expect 3 channel byte or single channel byte!\n"));

  cv::Mat tmp(im.size(),CV_8UC3);
  const int rows = im.rows;
  const int cols = im.cols;
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      uint8_t pix = im.at<uint8_t>(r,c);
      tmp.at<cv::Vec3b>(r,c) = cv::Vec3b(pix,pix,pix);
    }
  }

  im = tmp;
  return im;
}


ci::Vec3f ttrk::ConvertQuatToEulers(const ci::Quatf &quat){

  const float phi = (float)atan2(2 * (quat.w*quat.v[0] + quat.v[1]*quat.v[2]), 1 - 2 * (quat.v[0]*quat.v[0] + quat.v[1]*quat.v[1]));
  const float theta = (float)asin(2 * (quat.w*quat.v[1] - quat.v[2]*quat.v[0]));
  const float psi = (float)atan2(2 * (quat.w*quat.v[2] + quat.v[0]*quat.v[1]), 1 - 2 * (quat.v[1]*quat.v[1] + quat.v[2]*quat.v[2]));
  return ci::Vec3f(phi, theta, psi);

}

ci::Quatf ttrk::ConvertEulersToQuat(const ci::Vec3f &euler_angles){

  const float half_phi = euler_angles[0] / 2;
  const float half_theta = euler_angles[1] / 2;
  const float half_psi = euler_angles[2] / 2;
  const float q1 = (cos(half_phi)*cos(half_theta)*cos(half_psi)) + (sin(half_phi)*sin(half_theta)*sin(half_psi));
  const float q2 = (sin(half_phi)*cos(half_theta)*cos(half_psi)) - (cos(half_phi)*sin(half_theta)*sin(half_psi));
  const float q3 = (cos(half_phi)*sin(half_theta)*cos(half_psi)) + (sin(half_phi)*cos(half_theta)*sin(half_psi));
  const float q4 = (cos(half_phi)*cos(half_theta)*sin(half_psi)) - (sin(half_phi)*sin(half_theta)*cos(half_psi));

  return ci::Quatf(q1, q2, q3, q4);

}