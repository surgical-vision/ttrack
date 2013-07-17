#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  pose.translation_ = pose.translation_ + translation;

  //update the rotation
  sv::Quaternion rotation((float)jacobian.at<double>(3,0),cv::Vec3f((float)jacobian.at<double>(4,0),(float)jacobian.at<double>(5,0),(float)jacobian.at<double>(6,0)));
  pose.rotation_ = pose.rotation_ + rotation;
  pose.rotation_ = pose.rotation_.Normalize();

}

void PWP3D::ScaleJacobian(cv::Mat &jacobian, const int step_number) const {

  // experimental - SCALE THE STEP SIZE?
  static float scales[7] = { (float)1.0 , (float)0.8 , (float)0.7 , (float)0.5, (float)0.4 , (float)0.3, (float)0.1 };

  float scale = (float)3.0;
  //if( step_number > 6) scale = scales[6];
  //else scale = scales[step_number];

  jacobian = (float)0.00000001 * jacobian;
  for(int i=0;i<3;i++) jacobian.at<double>(i,0) *= 80 * scale;
  for(int i=3;i<7;i++) jacobian.at<double>(i,0) *= 0.4 * scale;

}

