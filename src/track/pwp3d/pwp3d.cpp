#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  std::cerr << "current translation: " << cv::Point3d(pose.translation_) << std::endl;
  std::cerr << "translation from jacobian: " << cv::Point3f(translation) << std::endl;
  pose.translation_ = pose.translation_ - translation;
  std::cerr << "translation after update: " << cv::Point3f(pose.translation_) << std::endl;

  //update the rotation
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));

  std::cerr << "current rotation: " << pose.rotation_ << std::endl;
  std::cerr << "rotation from jacobian: " << rotation << std::endl;
  pose.rotation_ = pose.rotation_ - rotation;
  //pose.rotation_ = pose.rotation_.Normalize();
  std::cerr << "rotation after update: " << pose.rotation_ << std::endl;


}

void PWP3D::ScaleJacobian(cv::Mat &jacobian) const {

  std::cerr << "Before scaling: " << jacobian << std::endl;

  jacobian = 0.00000000006 * jacobian;

  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));

  /*
  if(cv::norm(translation) > 1.0){
  //if(translation.dot(translation) > 0)
    cv::Vec3f t_translation = translation;
    cv::normalize(t_translation,translation);
    translation *= 5;
    //translation = cv::normalize(translation);
  }*/


  //translation = -translation;
  //translation[2] = 0;
  for(int i=0;i<3;i++) jacobian.at<double>(i,0) = translation[i]*70;
  
  
  //jacobian.at<double>(3,0) = 0;
  //jacobian.at<double>(4,0) = 0;
  //jacobian.at<double>(5,0) = 0;
  //jacobian.at<double>(6,0) = 0;
  /*
  jacobian.at<double>(3,0) = rotation.W();
  jacobian.at<double>(4,0) = rotation.X();
  jacobian.at<double>(5,0) = rotation.Y();
  jacobian.at<double>(6,0) = rotation.Z();
  */
  std::cerr << "After scaling: " << jacobian << std::endl;
}

