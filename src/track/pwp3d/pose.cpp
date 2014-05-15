#include "../../../include/track/pose.hpp"

using namespace ttrk;

const int PoseDerivs::NUM_VALS = 7;

Pose Pose::operator=(const cv::Mat &that){
  if(that.size() == cv::Size(1,9)){

    const cv::Vec3d translation(that(cv::Range(0,3),cv::Range::all()));     
    const cv::Vec3d velocity(that(cv::Range(3,6),cv::Range::all()));
    const cv::Vec3d rotation(that(cv::Range(6,9),cv::Range::all()));
    this->rotation_ = sv::Quaternion(rotation);
    this->translation_ = translation;
    this->translational_velocity_ = velocity;

  }else if(that.size() == cv::Size(9,1)){

    const cv::Vec3d translation(that(cv::Range::all(),cv::Range(0,3)));
    const cv::Vec3d velocity(that(cv::Range::all(),cv::Range(3,6)));
    const cv::Vec3d rotation(that(cv::Range::all(),cv::Range(6,9)));
    this->rotation_ = sv::Quaternion(rotation);
    this->translation_ = translation;
    this->translational_velocity_ = velocity;

  }else{
    throw(std::runtime_error("Error, invalid assignement to ttrk::Pose from cv::Mat. Dimensions do not match!\n"));
  }
  return *this;
}

ci::Matrix44d Pose::AsCiMatrix() const {

  cv::Vec3d euler = rotation_.EulerAngles();
  ci::Matrix44d ret = ci::Matrix44d::createRotation(ci::Vec3d(euler[0],euler[1],euler[2]));
  //ret.translate( ci::Vec3d(translation_[0],translation_[1],translation_[2]) );
  for(int i=0;i<3;i++) ret.at(i,3) = translation_[i];
  
  return ret;
  
}


Pose::operator cv::Mat() const {
  cv::Mat r(9,1,CV_32FC1);
  cv::Vec3d euler = rotation_.EulerAngles();    
  for(int i=0;i<3;i++){
    r.at<float>(i,0) = (float)translation_[i];
    r.at<float>(i+3,0) = (float)translational_velocity_[i];      
    r.at<float>(i+6,0) = (float)euler[i];
  }
  return r;  
}


void Pose::SetupFastDOFDerivs(double *data) const {

  data[0] = 1.0;
  data[1] = 0.0;
  data[2] = 0.0;
  data[3] = 0.0;
  data[4] = 1.0;
  data[5] = 0.0;
  data[6] = 0.0;
  data[7] = 0.0;
  data[8] = 1.0;

}


void Pose::GetFastDOFDerivs(double *data, double *point) const {

  data[9] =  (2*rotation_.Y()*point[2])-(2*rotation_.Z()*point[1]);
  data[10] = (2*rotation_.Z()*point[0])-(2*rotation_.X()*point[2]);
  data[11] = (2*rotation_.X()*point[1])-(2*rotation_.Y()*point[0]);
  data[12] = (2*rotation_.Y()*point[1])+(2*rotation_.Z()*point[2]);
  data[13] = (2*rotation_.Y()*point[0])-(4*rotation_.X()*point[1])-(2*rotation_.W()*point[2]);
  data[14] = (2*rotation_.Z()*point[0])+(2*rotation_.W()*point[1])-(4*rotation_.X()*point[2]);
  data[15] = (2*rotation_.X()*point[1])-(4*rotation_.Y()*point[0])+(2*rotation_.W()*point[2]);
  data[16] = (2*rotation_.X()*point[0])+(2*rotation_.Z()*point[2]);
  data[17] = (2*rotation_.Z()*point[1])-(2*rotation_.W()*point[0])-(4*rotation_.Y()*point[2]);
  data[18] = (2*rotation_.X()*point[2])-(2*rotation_.W()*point[1])-(4*rotation_.Z()*point[0]);
  data[19] = (2*rotation_.W()*point[0])-(4*rotation_.X()*point[1])+(2*rotation_.Y()*point[2]);
  data[20] = (2*rotation_.X()*point[0])+(2*rotation_.Y()*point[1]);

}


cv::Vec3d Pose::GetDOFDerivatives(const int dof, const cv::Vec3d &point_) const {

  //derivatives use the (x,y,z) from the initial reference frame not the transformed one so inverse the transformation
  cv::Vec3d point = point_ - translation_;
  point = rotation_.Inverse().RotateVector(point);

  //return the (dx/dL,dy/dL,dz/dL) derivative for the degree of freedom
  switch(dof){

  case 0: //x
    return cv::Vec3d(1,0,0);
  case 1: //y
    return cv::Vec3d(0,1,0);
  case 2: //z
    return cv::Vec3d(0,0,1);


  case 3: //qw
    return cv::Vec3d(
      (2*rotation_.Y()*point[2])-(2*rotation_.Z()*point[1]),
      (2*rotation_.Z()*point[0])-(2*rotation_.X()*point[2]),
      (2*rotation_.X()*point[1])-(2*rotation_.Y()*point[0]));

  case 4: //qx
    return cv::Vec3d(
      (2*rotation_.Y()*point[1])+(2*rotation_.Z()*point[2]),
      (2*rotation_.Y()*point[0])-(4*rotation_.X()*point[1])-(2*rotation_.W()*point[2]),
      (2*rotation_.Z()*point[0])+(2*rotation_.W()*point[1])-(4*rotation_.X()*point[2]));

  case 5: //qy
    return cv::Vec3d(
      (2*rotation_.X()*point[1])-(4*rotation_.Y()*point[0])+(2*rotation_.W()*point[2]),
      (2*rotation_.X()*point[0])+(2*rotation_.Z()*point[2]),
      (2*rotation_.Z()*point[1])-(2*rotation_.W()*point[0])-(4*rotation_.Y()*point[2]));
  case 6: //qz
    return cv::Vec3d(
      (2*rotation_.X()*point[2])-(2*rotation_.W()*point[1])-(4*rotation_.Z()*point[0]),
      (2*rotation_.W()*point[0])-(4*rotation_.X()*point[1])+(2*rotation_.Y()*point[2]),
      (2*rotation_.X()*point[0])+(2*rotation_.Y()*point[1]));

  default:
    throw std::runtime_error("Error, a value in the range 0-6 must be supplied");
  }
}
