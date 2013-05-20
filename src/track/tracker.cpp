#include "../../headers/track/tracker.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;

Tracker::Tracker(){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);

  if(!tracking_){
    if(!Init() || !InitKalmanFilter()) //do any custom initalisation in the virtual Init function
      return;
    tracking_ = true;
  }
  
  //track each model that we know about
  for(current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++ ){
    
    cv::Mat pose_estimate = localizer_->TrackTargetInFrame(*current_model_);
  
    UpdatePose(pose_estimate);
  
  }
}  






void Tracker::UpdatePose(const cv::Mat &pose_estimate){

  // get the prediction from the kalman filter

  // combine with the measurement

  // update the pose

}


bool Tracker::InitKalmanFilter(){

  for(int i = 0; i<tracked_models_.size(); i++){
  
    KalmanTracker &track = tracked_models_[i];
    cv::Mat initial_pose = track.model_->Pose();
    // set up kalman filter specific stuff
    
    track.filter_.init( 6 , 6 );

    
  }
    
  return true;

}





void Tracker::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  frame_.reset();
  frame_->Mat() = image;

}


boost::shared_ptr<cv::Mat> Tracker::GetPtrToFinishedFrame(){
  return frame_->Mat();
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}
  
