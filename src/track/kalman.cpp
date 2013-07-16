#include "../../headers/track/kalman.hpp"

using namespace ttrk;

KalmanTracker::KalmanTracker(const KalmanTracker &that){
  model_ = that.model_;
  pose_ = that.pose_;
}

void KalmanTracker::SetPose(const cv::Vec3f translation, const cv::Vec3f rotated_principal_axis) {

  pose_.translation_ = translation;
  pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),rotated_principal_axis);
  std::cout << "Initial translation = " << cv::Point3f(pose_.translation_) << "\n";
  std::cout << "Initial euler rotation = " << cv::Point3f(pose_.rotation_.EulerAngles()) << std::endl;
  std::cout << "Initial quaternion rotation = " << pose_.rotation_ << std::endl;

}

void KalmanTracker::UpdatePose(const Pose &pose_measurement) {

  pose_ = pose_measurement;
  return;
    
  const cv::Mat prediction = filter_.predict();
  std::cout << "prediction = " << prediction << std::endl;
  const cv::Mat estimation = filter_.correct((cv::Mat)pose_measurement);
  std::cout << "estimation = " << estimation << std::endl;
  //predict the measurement with the kalman filter
 
  cv::Mat position_prediction = *(cv::Mat_<float>(6, 1) << estimation.at<float>(0,0),estimation.at<float>(1,0),estimation.at<float>(2,0),estimation.at<float>(6,0),estimation.at<float>(7,0),estimation.at<float>(8,0) );
  std::cout << "position prediction: " << position_prediction << std::endl;
  pose_ = position_prediction;//position_prediction.clone(); 

}

void KalmanTracker::Init() {

  static float dt = 1;

  //params (x,y,z,dx,dy,dz,r1,r2,r3,,dr1,dr2,dr3) - euler angles allows linear parametrization
  filter_.init(12,6,0); //dynamic params, measurement params, control params

  filter_.statePost.at<float>(0) = 6.542;
  filter_.statePost.at<float>(1) = -3.981;
  filter_.statePost.at<float>(2) = 35.388;
  
  filter_.statePost.at<float>(3) = 0;
  filter_.statePost.at<float>(4) = 0;
  filter_.statePost.at<float>(5) = 0;

  filter_.statePost.at<float>(6) = -0.119;
  filter_.statePost.at<float>(7) = 0.54;
  filter_.statePost.at<float>(8) = -0.504;

  filter_.statePost.at<float>(9) = 0;
  filter_.statePost.at<float>(10) = 0;
  filter_.statePost.at<float>(11) = 0;

  cv::Mat state(6,1,CV_32F);
  cv::Mat process_noise(6,1,CV_32F);

  filter_.transitionMatrix = cv::Mat::eye(12,12,CV_32F);
  for(int i=3;i<6;i++){
    filter_.transitionMatrix.at<float>(i-3,i) = dt;
    filter_.transitionMatrix.at<float>(i+3,i+6) = dt;
  }
  std::cout << filter_.transitionMatrix << std::endl;

  filter_.measurementMatrix = cv::Mat::zeros(6,12,CV_32F);
  for(int i=0;i<3;i++){
    filter_.measurementMatrix.at<float>(i,i) = 1;
    filter_.measurementMatrix.at<float>(i+3,i+6) = 1;
  }

  std::cout << filter_.measurementMatrix << std::endl;

  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(1));
  
}
