#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  pose.translation_ = pose.translation_ - translation;
  
  //update the rotation
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));
  pose.rotation_ = pose.rotation_ - rotation;
  pose.rotation_ = pose.rotation_.Normalize();
  


}

void PWP3D::ScaleJacobian(cv::Mat &jacobian) const {

  jacobian = 0.000000001 * jacobian;

  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  sv::Quaternion rotation(jacobian.at<double>(3,0),cv::Vec3f(jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));

  if(cv::norm(translation) > 1.0)
    translation = cv::normalize(translation);
  translation *= 5;

  for(int i=0;i<3;i++) jacobian.at<double>(i,0) = translation[i];
  
  jacobian.at<double>(3,0) = rotation.W();
  jacobian.at<double>(4,0) = rotation.X();
  jacobian.at<double>(5,0) = rotation.Y();
  jacobian.at<double>(6,0) = rotation.Z();

}

