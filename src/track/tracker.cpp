#include "../include/headers.hpp"
#include "../../include/track/tracker.hpp"
#include "../../include/utils/helpers.hpp"
#include "../../include/utils/camera.hpp"

using namespace ttrk;

void Tracker::operator()(boost::shared_ptr<sv::Frame> image, const bool found){
  Run(image, found);
} 

void Tracker::Run(boost::shared_ptr<sv::Frame> image, const bool found){

    SetHandleToFrame(image);

    if (!found){
        //tracking_ = false;
        return;
    }

    if (!tracking_){

        //need this as init constructs new tracking models
        tracked_models_.clear(); //get rid of anything we were tracking before

        if (!Init() || !InitTemporalModels()) //do any custom initalisation in the virtual Init function
            return;
        tracking_ = true;

    }

    //track each model that we know about
    for (current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++){

        localizer_->TrackTargetInFrame(current_model_->model, frame_);
        current_model_->temporal_tracker->UpdatePoseWithMotionModel(current_model_->model);

    }

}


bool Tracker::InitTemporalModels(){

  for(auto i = tracked_models_.begin(); i!=tracked_models_.end();i++)
    i->temporal_tracker->Init();
      
  return true;

}

void Tracker::GetTrackedModels(std::vector <boost::shared_ptr<Model> > &models){
      
  std::transform(tracked_models_.begin(), tracked_models_.end(), std::back_inserter(models), [](TemporalTrackedModel &m) { return m.model; });  

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
  
