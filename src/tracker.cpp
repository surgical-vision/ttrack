#include "../headers/tracker.hpp"

using namespace ttrk;

Tracker::Tracker(){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  frame_ = image;
  
  if(!tracking_) 
    Init();

  //detect

}
  
void Tracker::Init(){
  //pose init
}

boost::shared_ptr<cv::Mat> Tracker::GetPtrToFinishedFrame(){
  return frame_;
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}
  
