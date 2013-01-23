#include "../headers/tracker.hpp"

using namespace ttrk;

Tracker::Tracker(){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);

  if(!tracking_ && !Init())
    return;
    
  circle(*frame_,cv::Point(400,100),20,cv::Scalar(200,182,233),-1);
  
}  

void Tracker::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  frame_.reset();
  frame_ = image;

}

/*bool Tracker::Init(){
  //pose init
  bool found = FindConnectedRegions();

  InitPoseFromMOITensor();

}*/

boost::shared_ptr<cv::Mat> Tracker::GetPtrToFinishedFrame(){
  return frame_;
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}
  
