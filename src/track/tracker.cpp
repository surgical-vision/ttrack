#include "../../headers/track/tracker.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;

void Tracker::operator()(boost::shared_ptr<sv::Frame> image, const bool found){
  
  SetHandleToFrame(image);

  if(!found){
    //tracking_ = false;
    return;
  }

  if(!tracking_){
    tracked_models_.clear(); //get rid of anything we were tracking before
    if(!Init() || !InitKalmanFilter()) //do any custom initalisation in the virtual Init function
      return;
    tracking_ = true;
  }
  
  //track each model that we know about
  for(current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++ ){
    
    Pose pose_measurement = localizer_->TrackTargetInFrame(*current_model_,image);
  
    current_model_->UpdatePose(pose_measurement);
    
    DrawModelOnFrame(*current_model_,frame_->Mat());
  
  }
}  

bool Tracker::InitKalmanFilter(){

  for(auto i = tracked_models_.begin(); i!=tracked_models_.end();i++)
    i->Init();
      
  return true;

}

void Tracker::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){
  frame_ = image;
}

boost::shared_ptr<sv::Frame> Tracker::GetPtrToFinishedFrame(){
  return frame_;
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}

Tracker::Tracker(){}

Tracker::~Tracker(){}
  
