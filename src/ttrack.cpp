#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
using namespace ttrk;

void TTrack::Run(){

  // grab the first frame either from video or from image file
  //ptr owned solely by ttrack
  v_frame_ = GetPtrToFrame();

  // clone it and classify collecting the result in c_frame_
  detector_(v_frame_);
  c_frame_ = detector_.GetPtrToClassified();

  // do pose initialisation
  tracker_.Init(c_frame_);

  while(detector_.Found()){

    // start processing the classified frame
    boost::thread TrackThread(boost::ref(tracker_),c_frame_);

    //grab a pointer to the next frame and run rf detection on it
    //NOTE - if (v_frame_ == NULL) found=false so loop will end 
    v_frame_ = GetPtrToFrame();
    boost::thread DetectThread(boost::ref(detector_),v_frame_);
    
    //synchronise the threads to avoid overwriting the classified image
    TrackThread.join();
    DetectThread.join();

    //save the images currently held by the classifier and the tracker.
#ifdef DEBUG
    SaveDebug();
#endif
    
    //save the output images/video (this->c_frame_) with the drawn pose on top.
    SaveFrame();

    //get a pointer to processed frame from detect
    c_frame_ = detector_.GetPtrToClassified();

  }

}

/*
void TTrack::Run(){

  detector_(GetPtrToFrame()); // { 

  tracker_.Init(frame);

  while(detector_.Found()){
    
    boost::thread TrackThread(boost::ref(tracker_),GetPtrToClassified());
    boost::thread DetectThread(boost::ref(detector_),GetPtrToFrame());
    

    TrackThread.join();
    DetectThread.join();
  
    SaveFrame();
    
    CleanUp();

    }
*/
  


void TTrack::RunVideo(){
  
  handler_ = new VideoHandler(root_dir_ + "/data/",root_dir_);
  Run();

}


void TTrack::RunImages(){

  handler_ = new ImageHandler(root_dir_ + "/data/",root_dir_);
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

  cv::Mat *m = handler_->GetPtrToFrame();
  if(m!=0)
    return boost::shared_ptr<cv::Mat>(m);
  else{
    return boost::shared_ptr<cv::Mat>();
  }

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

void TTrack::SetUpDirectoryTree(const std::string &root_dir){

  root_dir_ = root_dir; //the directory where data is
  
  detector_.Setup(root_dir);

}
