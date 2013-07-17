#include "../../headers/track/kalman.hpp"

using namespace ttrk;

KalmanTracker::KalmanTracker(const KalmanTracker &that){
  model_ = that.model_;
  pose_ = that.pose_;
}

void KalmanTracker::SetPose(const cv::Vec3f translation, const cv::Vec3f rotated_principal_axis) {

  pose_.translation_ = translation;
  pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),rotated_principal_axis);
  pose_.translational_velocity_ = cv::Vec3f(0,0,0);

}

void KalmanTracker::UpdatePose(const Pose &pose_measurement) {

  //predict the measurement with the kalman filter
  const cv::Mat prediction = filter_.predict();
  const cv::Mat estimation = filter_.correct((cv::Mat)pose_measurement);
  
  cv::Mat position_prediction = *(cv::Mat_<float>(9, 1) << estimation.at<float>(0,0),estimation.at<float>(1,0),estimation.at<float>(2,0),estimation.at<float>(3,0),estimation.at<float>(4,0),estimation.at<float>(5,0),estimation.at<float>(6,0),estimation.at<float>(7,0),estimation.at<float>(8,0) );
  pose_ = position_prediction;

}

void KalmanTracker::Init() {

  static float dt = 1;

  //params (x,y,z,dx,dy,dz,r1,r2,r3,,dr1,dr2,dr3) - euler angles allows linear parametrization
  filter_.init(12,9,0); //dynamic params, measurement params, control params

  cv::Mat state(6,1,CV_32F);
  cv::Mat process_noise(6,1,CV_32F);

  filter_.transitionMatrix = cv::Mat::eye(12,12,CV_32F);
  for(int i=3;i<6;i++){
    filter_.transitionMatrix.at<float>(i-3,i) = dt;
    filter_.transitionMatrix.at<float>(i+3,i+6) = dt;
  }

  filter_.measurementMatrix = cv::Mat::zeros(9,12,CV_32F);
  for(int i=0;i<9;i++)
    filter_.measurementMatrix.at<float>(i,i) = 1;
  

  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-4));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(1));
  
}
