#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());

  std::cerr << "current translation: " << cv::Point3d(pose.translation_) << std::endl;
  std::cerr << "translation from jacobian: " << cv::Point3f(translation) << std::endl;
  pose.translation_ = pose.translation_ + translation;
  std::cerr << "translation after update: " << cv::Point3f(pose.translation_) << std::endl;

  //update the rotation
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));
  //sv::Quaternion rotation(0.0,cv::Vec3f(0.0,0.0,0.0));

  std::cerr << "current rotation: " << pose.rotation_ << std::endl;
  std::cerr << "rotation from jacobian: " << rotation << std::endl;
  pose.rotation_ = pose.rotation_ + rotation;

  //static int n=0;
  //if( (n%5)==0 && n>0)
  pose.rotation_ = pose.rotation_.Normalize();
  //n++;

  std::cerr << "rotation after update: " << pose.rotation_ << std::endl;


}

void PWP3D::ScaleJacobian(cv::Mat &jacobian, const int step_number) const {

  static float scales[7] = { 1.0 , 0.8 , 0.7 , 0.5, 0.4 , 0.3, 0.1 };

  float scale = 3;
  //if( step_number > 6) scale = scales[6];
  //else scale = scales[step_number];

  std::cerr << "Before scaling: " << jacobian << std::endl;

  jacobian = 0.00000001 * jacobian;
  for(int i=0;i<3;i++) jacobian.at<double>(i,0) *= 50 * scale;
  for(int i=3;i<7;i++) jacobian.at<double>(i,0) *= 0.4 * scale;

  std::cerr << "After scaling: " << jacobian << std::endl;

}

