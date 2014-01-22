#include "../../include/track/tracker.hpp"
#include "../../include/utils/helpers.hpp"
#include "../../include/utils/camera.hpp"

using namespace ttrk;

void Tracker::operator()(boost::shared_ptr<sv::Frame> image, const bool found){
  

  SetHandleToFrame(image);

  if(!found){
    //tracking_ = false;
    return;
  }

  if(!tracking_){

    //need this as init constructs new tracking models
    tracked_models_.clear(); //get rid of anything we were tracking before

    if(!Init() || !InitKalmanFilter()) //do any custom initalisation in the virtual Init function
      return;
    tracking_ = true;

  }
  
  //track each model that we know about
  for(current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++ ){
    
    try{
      Pose pose_measurement = localizer_->TrackTargetInFrame(*current_model_,frame_);
      current_model_->UpdatePose(pose_measurement);
    }catch(std::exception &e){
      std::cerr << "ERROR IN THIS UPDATE!\n";
      std::cerr << e.what() << "\n";
      continue;
    }

    

    //if( localizer_->ModelInFrame( *current_model_, frame_->GetClassificationMapROI() ))
    DrawModelOnFrame(*current_model_,frame_->GetImage());
    //else
      //tracking_ = false;
  
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
  
