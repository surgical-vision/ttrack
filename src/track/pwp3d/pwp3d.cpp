#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  translation[0] *= -1;
  translation[1] *= -1;
  translation[2] *= -1;
  std::cerr << "current translation: " << cv::Point3d(pose.translation_) << std::endl;
  std::cerr << "translation from jacobian: " << cv::Point3f(translation) << std::endl;
  pose.translation_ = pose.translation_ - translation;
  std::cerr << "translation after update: " << cv::Point3f(pose.translation_) << std::endl;

  //update the rotation
  //sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));
  sv::Quaternion rotation(0.0,cv::Vec3f(0.0,0.0,0.0));

  std::cerr << "current rotation: " << pose.rotation_ << std::endl;
  std::cerr << "rotation from jacobian: " << rotation << std::endl;
  pose.rotation_ = pose.rotation_ - rotation;
  //pose.rotation_ = pose.rotation_.Normalize();
  std::cerr << "rotation after update: " << pose.rotation_ << std::endl;


}

void PWP3D::ScaleJacobian(cv::Mat &jacobian) const {

  std::cerr << "Before scaling: " << jacobian << std::endl;

  jacobian = 0.000000005 * jacobian;
  for(int i=0;i<3;i++) jacobian.at<double>(i,0) *= 200;

  std::cerr << "After scaling: " << jacobian << std::endl;

}

