#include "../headers/tracker.hpp"

using namespace ttrk;

Tracker::Tracker(){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);

  if(!tracking_) 
    Init();
  //track

  circle(*frame_,cv::Point(400,100),20,cv::Scalar(200,182,233),-1);
  
}  

void Tracker::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  frame_.reset();
  frame_ = image;

}

void Tracker::ResetHandleToFrame(){

  frame_.reset();

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
  
