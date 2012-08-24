#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <vector>
#include <cassert>

using namespace ttrk;

void TTrack::Run(){

  // grab the first frame either from video or from image file
  v_frame_ = GetPtrToFrame();

  // clone it and classify collecting the result in c_frame_
  detector_(v_frame_);
  c_frame_ = detector_.GetPtrToClassified();

  // do pose initialisation
  tracker_.Init(c_frame_);


  while(detector_.Found()){

    // start processing the classified frame
    boost::thread TrackThread(tracker_,c_frame_);
    
    //grab a pointer to the next frame and run rf detection on it
    //NOTE - if (v_frame_ == NULL) found=false so loop will end 
    v_frame_ = GetPtrToFrame();
    boost::thread DetectThread(detector_,v_frame_); 

    //synchronise the threads to avoid overwriting the classified image
    TrackThread.join();
    DetectThread.join();

    //save the images currently held by the classifier and the tracker.
#ifdef DEBUG
    SaveDebug();
#endif

    //save the output images/video with the drawn pose on top.
    SaveFrame();

    //get a pointer to processed frame from detect
    c_frame_ = detector_.GetPtrToClassified(); 
    break;
  }

}

void TTrack::RunVideo(const std::string &root_dir){
  
  handler_ = new VideoHandler(root_dir,root_dir);

  Run();

}


void TTrack::RunImages(const std::string &root_dir){

  handler_ = new ImageHandler(root_dir,root_dir);
 
  Run();

}

void TTrack::SaveFrame(){

  //draws the model at the current pose on c_frame_
  DrawModel();
  
  handler_->SavePtrToFrame(c_frame_.get());

}


void TTrack::DrawModel(){

}

boost::shared_ptr<cv::Mat> TTrack::GetPtrToFrame(){
  return boost::shared_ptr<cv::Mat>(handler_->GetPtrToFrame());
}


void TTrack::SaveDebug() const {
  
  //iterate through all images, push to vector
  //send to iamge handler

}


/******** SETUP CODE ***********/


bool TTrack::constructed_ = false;
TTrack *TTrack::instance_ = 0;

TTrack::TTrack():handler_(0),c_frame_(new cv::Mat),v_frame_(new cv::Mat){}
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
  c_frame_ = that.c_frame_;
  v_frame_ = that.v_frame_;

  return *this;

}

TTrack &TTrack::Instance(){

  if(!constructed_){
    instance_ = new TTrack();
    constructed_ = true;
  }

  return *instance_;

}


