#include "../../../headers/track/pose.hpp"

using namespace ttrk;

Pose Pose::operator=(const cv::Mat &that){
  if(that.size() == cv::Size(1,9)){

    const cv::Vec3f translation(that(cv::Range(0,3),cv::Range::all()));     
    const cv::Vec3f velocity(that(cv::Range(3,6),cv::Range::all()));
    const cv::Vec3f rotation(that(cv::Range(6,9),cv::Range::all()));
    this->rotation_ = sv::Quaternion(rotation);
    this->translation_ = translation;
    this->translational_velocity_ = velocity;

  }else if(that.size() == cv::Size(9,1)){

    const cv::Vec3f translation(that(cv::Range::all(),cv::Range(0,3)));
    const cv::Vec3f velocity(that(cv::Range::all(),cv::Range(3,6)));
    const cv::Vec3f rotation(that(cv::Range::all(),cv::Range(6,9)));
    this->rotation_ = sv::Quaternion(rotation);
    this->translation_ = translation;
    this->translational_velocity_ = velocity;

  }else{
    throw(std::runtime_error("Error, invalid assignement to ttrk::Pose from cv::Mat. Dimensions do not match!\n"));
  }
  return *this;
}



Pose::operator cv::Mat() const {
  cv::Mat r(9,1,CV_32FC1);
  cv::Vec3f euler = rotation_.EulerAngles();    
  for(int i=0;i<3;i++){
    r.at<float>(i,0) = translation_[i];
    r.at<float>(i+3,0) = translational_velocity_[i];      
    r.at<float>(i+6,0) = euler[i];
  }
  return r;  
}



cv::Vec3f Pose::GetDOFDerivatives(const int dof, const cv::Vec3f &point_) const {

  //derivatives use the (x,y,z) from the initial reference frame not the transformed one so inverse the transformation
  cv::Vec3f point = point_ - translation_;
  point = rotation_.Inverse().RotateVector(point);

  //return the (dx/dL,dy/dL,dz/dL) derivative for the degree of freedom
  switch(dof){

  case 0: //x
    return cv::Vec3f(1,0,0);
  case 1: //y
    return cv::Vec3f(0,1,0);
  case 2: //z
    return cv::Vec3f(0,0,1);


  case 3: //qw
    return cv::Vec3f(
      (2*rotation_.Y()*point[2])-(2*rotation_.Z()*point[1]),
      (2*rotation_.Z()*point[0])-(2*rotation_.X()*point[2]),
      (2*rotation_.X()*point[1])-(2*rotation_.Y()*point[0]));

  case 4: //qx
    return cv::Vec3f(
      (2*rotation_.Y()*point[1])+(2*rotation_.Z()*point[2]),
      (2*rotation_.Y()*point[0])-(4*rotation_.X()*point[1])-(2*rotation_.W()*point[2]),
      (2*rotation_.Z()*point[0])+(2*rotation_.W()*point[1])-(4*rotation_.X()*point[2]));

  case 5: //qy
    return cv::Vec3f(
      (2*rotation_.X()*point[1])-(4*rotation_.Y()*point[0])+(2*rotation_.W()*point[2]),
      (2*rotation_.X()*point[0])+(2*rotation_.Z()*point[2]),
      (2*rotation_.Z()*point[1])-(2*rotation_.W()*point[0])-(4*rotation_.Y()*point[2]));
  case 6: //qz
    return cv::Vec3f(
      (2*rotation_.X()*point[2])-(2*rotation_.W()*point[1])-(4*rotation_.Z()*point[0]),
      (2*rotation_.W()*point[0])-(4*rotation_.X()*point[1])+(2*rotation_.Y()*point[2]),
      (2*rotation_.X()*point[0])+(2*rotation_.Y()*point[1]));

  default:
    throw std::runtime_error("Error, a value in the range 0-6 must be supplied");
  }
}
