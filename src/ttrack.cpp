#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
using namespace ttrk;


void TTrack::Run(){

  detector_( GetPtrToNewFrame() ); 

  while(detector_.Found()){
    
    boost::thread TrackThread(boost::ref(tracker_), GetPtrToClassifiedFrame() );
    boost::thread DetectThread(boost::ref(detector_), GetPtrToNewFrame() );
    
    TrackThread.join();
    DetectThread.join();

#ifdef DEBUG
    SaveFrame();
#endif

    CleanUp();

  }
  
}  

void TTrack::Train(){
  detector_.Train();
}


void TTrack::RunVideo(){
  
  
  tracker_.Tracking(true);
  handler_ = new VideoHandler(root_dir_ + "/data/",root_dir_);
  Run();

}


void TTrack::RunImages(){

  tracker_.Tracking(false);
  handler_ = new ImageHandler(root_dir_ + "/data/",root_dir_);
  Run();

}

void TTrack::CleanUp(){

  delete tracker_.GetPtrToFinishedFrame();

}

void TTrack::SaveFrame(){

  //draws the model at the current pose on c_frame_
  cv::Mat *frame = tracker_.GetPtrToFinishedFrame();

  DrawModel(frame);
  
  handler_->SavePtrToFrame(frame);

}


void TTrack::DrawModel(cv::Mat *frame){

}

cv::Mat *TTrack::GetPtrToNewFrame(){
  
  frame_ = handler_->GetPtrToNewFrame(); //now point at new frame, discarding old
  return frame_;

}


cv::Mat *TTrack::GetPtrToClassifiedFrame(){

  frame_ = detector_.GetPtrToClassifiedFrame();
  return frame_;

}

void TTrack::SaveDebug() const {
  
  //iterate through all images, push to vector
  //send to iamge handler

}


/******** SETUP CODE ***********/


bool TTrack::constructed_ = false;
TTrack *TTrack::instance_ = 0;

TTrack::TTrack():handler_(0){}
TTrack::~TTrack(){
  delete handler_;
}

void TTrack::Destroy(){

  delete instance_;
  instance_ = 0;
  constructed_ = false;

}


TTrack::TTrack(const TTrack &that){
  *this = that;
}

TTrack &TTrack::operator=(const TTrack &that){

  // check for self-assignment
  if(this == &that) return *this;
  tracker_ = that.tracker_;
  detector_ = that.detector_;
  frame_ = that.frame_;

  return *this;

}

TTrack &TTrack::Instance(){
  if(!constructed_){
    instance_ = new TTrack();
    constructed_ = true;
  }
  return *instance_;
}

void TTrack::SetUpDirectoryTree(const std::string &root_dir){

  root_dir_ = root_dir; //the directory where data is
  
  detector_.Setup(root_dir);

}
