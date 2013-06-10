#include "../../headers/track/kalman.hpp"

using namespace ttrk;

KalmanTracker::KalmanTracker(const KalmanTracker &that){
  model_ = that.model_;
  pose_ = that.pose_;
}

void KalmanTracker::SetPose(const cv::Vec3f translation, const cv::Vec3f rotated_principal_axis) {

  pose_.translation_ = translation;
  pose_.rotation_ = sv::Quaternion::FromVectorToVector(model_->PrincipalAxis(),rotated_principal_axis);

}
