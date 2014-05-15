#include "../../include/track/kalman.hpp"

using namespace ttrk;

KalmanTracker::KalmanTracker(const KalmanTracker &that){
  model_ = that.model_;
  pose_ = that.pose_;
  save_file_ = that.save_file_;
  filter_ = that.filter_;
  that.temporary_update_pose_.copyTo(temporary_update_pose_);
  //
}

KalmanTracker &KalmanTracker::operator=(const KalmanTracker &kt){

  KalmanTracker copy(kt);

  std::swap(model_,copy.model_);
  std::swap(pose_,copy.pose_);
  std::swap(save_file_,copy.save_file_);
  std::swap(filter_,copy.filter_);
  std::swap(temporary_update_pose_,copy.temporary_update_pose_);
  
  return *this;

}

void KalmanTracker::SetPose(const cv::Vec3d translation, const cv::Vec3d rotated_principal_axis) {

  pose_.translation_ = translation;
  //pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),rotated_principal_axis);
  if( (model_->PrincipalAxis()[0] > 0 && rotated_principal_axis[0] > 0) || (model_->PrincipalAxis()[0] < 0 && rotated_principal_axis[0] < 0)){
    pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),rotated_principal_axis);
  }else{
    cv::Vec3d rotated_principal_axis_ = rotated_principal_axis;
    if(rotated_principal_axis_[2] == 0) rotated_principal_axis_[2] = 0.000001;
    pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),cv::Vec3d(0,0,rotated_principal_axis_[2]/std::abs(rotated_principal_axis_[2])));
    pose_.rotation_.Normalize();
    pose_.rotation_ = sv::Quaternion::FromVectorToVector(cv::Vec3d(0,0,rotated_principal_axis_[2]/std::abs(rotated_principal_axis_[2])),cv::Vec3d(rotated_principal_axis_[0],0,0)) * pose_.rotation_;
    pose_.rotation_.Normalize();
    pose_.rotation_ = sv::Quaternion::FromVectorToVector(cv::Vec3d(rotated_principal_axis_[0],0,0),rotated_principal_axis_) * pose_.rotation_;
    pose_.rotation_.Normalize();
  }

  //pose_.rotation_ = pose_.rotation_ + sv::Quaternion(boost::math::quaternion<double>(0.01,-0.011*rotated_principal_axis[0],-0.01*rotated_principal_axis[1],-0.01*rotated_principal_axis[2]));
  pose_.translational_velocity_ = cv::Vec3d(0,0,0);

}

void KalmanTracker::SetPose(const Pose &p){

  pose_ = p;

}

void KalmanTracker::UpdatePose(const Pose &pose_measurement) {

  const cv::Mat prediction = filter_.predict(); //compute prior P(w_{t}|x_{1},...,x_{t-1}) (using Chapman-Kolomogorov e.q.)
  const cv::Mat estimation = filter_.correct((cv::Mat)pose_measurement); //compute posterior by combining the measurement likelihood with the prior
  
  cv::Mat position_prediction = estimation(cv::Range(0, 9), cv::Range::all());// .clone();// *(cv::Mat_<float>(9, 1) << estimation.at<float>(0, 0), estimation.at<float>(1, 0), estimation.at<float>(2, 0), estimation.at<float>(3, 0), estimation.at<float>(4, 0), estimation.at<float>(5, 0), estimation.at<float>(6, 0), estimation.at<float>(7, 0), estimation.at<float>(8, 0));
  pose_ = position_prediction;

}

void KalmanTracker::Init() {

  static float dt = 1;

  //params (x,y,z,dx,dy,dz,r1,r2,r3,dr1,dr2,dr3) - euler angles allows linear parametrization
  filter_.init(12,9,0); //dynamic params, measurement params, control params

  filter_.transitionMatrix = cv::Mat::eye(12,12,CV_32F);
  for(int i=3;i<6;i++){
    filter_.transitionMatrix.at<float>(i-3,i) = dt;
    filter_.transitionMatrix.at<float>(i+3,i+6) = dt;
  }

  filter_.measurementMatrix = cv::Mat::zeros(9,12,CV_32F);
  for(int i=0;i<9;i++)
    filter_.measurementMatrix.at<float>(i,i) = 1;
  

  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(2e-3)); //uncertainty in the model
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-3)); //uncertainty in the measurement
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(1));
  
}
