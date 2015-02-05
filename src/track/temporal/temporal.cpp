#include "../../../include/ttrack/track/temporal/temporal.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

#include <cinder/app/App.h>

using namespace ttrk;

void KalmanFilterTracker::Init(std::vector<float> &start_pose) {

  static float dt = 1.0f/25; //frame rate of 25Hz

  //params (x,y,z,dx,dy,dz,r1,r2,r3,dr1,dr2,dr3) - euler angles allows linear parametrization
  filter_.init(12, 6, 0, CV_32F); //dynamic params, measurement params, control params

  filter_.transitionMatrix = cv::Mat::eye(12, 12, CV_32F);
  for (int i = 3; i<6; i++){
    filter_.transitionMatrix.at<float>(i - 3, i) = dt;
    filter_.transitionMatrix.at<float>(i + 3, i + 6) = dt;
  }

  filter_.measurementMatrix = cv::Mat::zeros(6, 12, CV_32F);
  for (int i = 0; i < 3; i++){
    filter_.measurementMatrix.at<float>(i, i) = 1;
    filter_.measurementMatrix.at<float>(i + 3, i + 6) = 1;
  }

  cv::Vec3f eulers = CiToCv<float>(ConvertQuatToEulers(ci::Quatf(start_pose[3], start_pose[4], start_pose[5], start_pose[6])));

  filter_.statePost.at<float>(0) = start_pose[0];
  filter_.statePost.at<float>(1) = start_pose[1];
  filter_.statePost.at<float>(2) = start_pose[2];
  filter_.statePost.at<float>(3) = 0;
  filter_.statePost.at<float>(4) = 0;
  filter_.statePost.at<float>(5) = 0;
  filter_.statePost.at<float>(6) = eulers[0];
  filter_.statePost.at<float>(7) = eulers[1];
  filter_.statePost.at<float>(8) = eulers[2];
  filter_.statePost.at<float>(9) = 0;
  filter_.statePost.at<float>(10) = 0;
  filter_.statePost.at<float>(11) = 0;
    
  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(1)); //uncertainty in the model
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-2)); //uncertainty in the measurement
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(1));

}


void KalmanFilterTracker::UpdatePoseWithMotionModel(boost::shared_ptr<Model> model){

  ci::Matrix44f pose = model->GetBasePose();
  ci::Matrix33f rotation = pose.subMatrix33(0, 0);
  
  ci::Quatf rq(rotation);
  
  ci::Vec3f eulers = ConvertQuatToEulers(rq);

  ci::Vec3f translation = pose.getTranslate().xyz();

  std::vector<float> measurement_vector = { translation[0], translation[1], translation[2], eulers[0], eulers[1], eulers[2]};

  cv::Mat pose_measurement;
  for (auto i = 0; i < measurement_vector.size(); ++i){
    pose_measurement.push_back(measurement_vector[i]);
  }

  const cv::Mat prediction = filter_.predict(); //compute prior P(w_{t}|x_{1},...,x_{t-1}) (using Chapman-Kolomogorov e.q.)
  const cv::Mat estimation = filter_.correct(pose_measurement); //compute posterior by combining the measurement likelihood with the prior

  //don't use the velocities in this part
  ci::Vec3f updated_translation(estimation.at<float>(0), estimation.at<float>(1), estimation.at<float>(2));
  cv::Vec3f updated_rotation_as_euler(estimation.at<float>(6), estimation.at<float>(7), estimation.at<float>(8));

  model->SetBasePose(Pose(ci::Quatf(ttrk::ConvertEulersToQuat(CvToCi<float>(updated_rotation_as_euler))), updated_translation));
  
}

