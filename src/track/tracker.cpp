#include <cinder/app/App.h>

#include "../include/headers.hpp"
#include "../../include/track/tracker.hpp"
#include "../../include/utils/helpers.hpp"
#include "../../include/utils/camera.hpp"

using namespace ttrk;

Tracker::Tracker(const std::string &model_parameter_file, const std::string &results_dir) : model_parameter_file_(model_parameter_file), results_dir_(results_dir), tracking_(false) {}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<sv::Frame> image, const bool found){
  
  Run(image, found);

}

void Tracker::RunStep(){

  for (current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++){

    localizer_->TrackTargetInFrame(current_model_->model, frame_);
    ci::app::console() << "Pose after GD step = " << current_model_->model->GetBasePose() << std::endl;
    
  }

}


void Tracker::Run(boost::shared_ptr<sv::Frame> image, const bool found){

  SetHandleToFrame(image);

  if (!found || image == nullptr){
    //tracking_ = false;
    return;
  }

  if (!tracking_){

    //need this as init constructs new tracking models
    tracked_models_.clear(); //get rid of anything we were tracking before

    if (!Init() || !InitTemporalModels()) //do any custom initalisation in the virtual Init function
      return;
    tracking_ = true;

    for (current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++){
      ci::app::console() << "Pose after init = " << current_model_->model->GetBasePose() << std::endl;
    }

  }
  else{

    for (current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++){
      //current_model_->temporal_tracker->UpdatePoseWithMotionModel(current_model_->model);
      ci::app::console() << "Pose after temporal update = " << current_model_->model->GetBasePose() << std::endl;
    }

  }

  RunStep();

}


bool Tracker::InitTemporalModels(){

  for (auto i = tracked_models_.begin(); i != tracked_models_.end(); i++){
    std::vector<float> pose;
    i->model->GetPose(pose);
    i->temporal_tracker->Init(pose);
  }
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
