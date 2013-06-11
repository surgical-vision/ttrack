#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));

  std::cerr << "update translations = " << translation[0] << "," << translation[1] << "," << translation[2] <<"\n";
  std::cerr << "update rotation = " << rotation.W() << "," << rotation.X() << "," << rotation.Y() << "," << rotation.Z() << "\n";
  
  
  std::cerr << "translation before = " << pose.translation_[0] << "," << pose.translation_[1] << "," << pose.translation_[2] <<"\n";
  std::cerr << "rotation before = " << pose.rotation_.W() << "," << pose.rotation_.X() << "," << pose.rotation_.Y() << "," << pose.rotation_.Z() << "\n";
  

  pose.rotation_ = rotation * pose.rotation_;
  pose.translation_ = translation + pose.translation_;

  std::cerr << "translation after = " << pose.translation_[0] << "," << pose.translation_[1] << "," << pose.translation_[2] <<"\n";
  std::cerr << "rotation after = " << pose.rotation_.W() << "," << pose.rotation_.X() << "," << pose.rotation_.Y() << "," << pose.rotation_.Z() << "\n";
  

}

void PWP3D::ScaleJacobian(cv::Mat &jacobian) const {

  std::cerr << "jacobian = " << jacobian << std::endl;
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));
  rotation = rotation.Normalize();

  translation = cv::normalize(translation);
  translation *= 10;

  for(int i=0;i<3;i++) jacobian.at<double>(i,0) = translation[i];
  
  jacobian.at<double>(3,0) = rotation.W();
  jacobian.at<double>(4,0) = rotation.X();
  jacobian.at<double>(5,0) = rotation.Y();
  jacobian.at<double>(6,0) = rotation.Z();

}

