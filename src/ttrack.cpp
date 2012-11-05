#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>

using namespace ttrk;

void TTrack::SetUp(std::string root_dir, const ClassifierType classifier_type, const TrainType train_type){
  
  if(!boost::filesystem::exists(boost::filesystem::path(root_dir)))
    throw std::runtime_error("Error, directory " + root_dir + " does not exist.\n");

  //set the shared_ptr containing the directory where data is
  root_dir_.reset(new std::string(root_dir)); 
  
  try{
    
    //if train type is NA, training is skipped
    detector_ = new Detect(root_dir_,classifier_type,train_type);
    tracker_ = new Tracker;

  }catch(std::bad_alloc &e){
    std::cerr << "Error, memory error. Could not construct detector/tracker.\n" << e.what();
    std::cerr << "Exiting...\n";
    exit(1);
  }catch(std::exception &e){
    std::cerr << "Error, caught exception.\n" <<  e.what();
    std::cerr << "Exiting...\n";
    exit(1);
  }

  
}

void TTrack::Run(){

  (*detector_)( GetPtrToNewFrame() ); 

  while(detector_->Found()){
    
    boost::thread TrackThread(boost::ref(*tracker_), GetPtrToClassifiedFrame() );
    boost::thread DetectThread(boost::ref(*detector_), GetPtrToNewFrame() );
    
    TrackThread.join();
    DetectThread.join();

#ifdef DEBUG
    SaveFrame();
#endif

    CleanUp();

  }
  
}  

void TTrack::TestDetector(const std::string &infile, const std::string &outfile){

  assert(0);

}

void TTrack::RunVideo(const std::string &video_url){
  
  tracker_->Tracking(true);
  handler_ = new VideoHandler(*root_dir_ + video_url, *root_dir_);
  Run();
  delete handler_;
}

void TTrack::RunImages(const std::string &image_url){

  tracker_->Tracking(false);
  handler_ = new ImageHandler(*root_dir_ + image_url, *root_dir_);
  Run();
  delete handler_;

}

void TTrack::SaveFrame(){

  //draws the model at the current pose on c_frame_
  cv::Mat *frame = tracker_->GetPtrToFinishedFrame();

  DrawModel(frame);
  
  handler_->SavePtrToFrame(frame);

}

void TTrack::SaveDebug() const {
  
  //iterate through all images, push to vector
  //send to iamge handler

}

void TTrack::DrawModel(cv::Mat *frame){

}

cv::Mat *TTrack::GetPtrToNewFrame(){
  
  frame_ = handler_->GetPtrToNewFrame(); //now point at new frame, discarding old
  return frame_;

}

cv::Mat *TTrack::GetPtrToClassifiedFrame(){

  frame_ = detector_->GetPtrToClassifiedFrame();
  return frame_;

}

/******** SETUP CODE ***********/

bool TTrack::constructed_ = false;

TTrack *TTrack::instance_ = 0;

TTrack::TTrack():tracker_(0),detector_(0),handler_(0),frame_(0){}

TTrack::~TTrack(){
  delete handler_;
  handler_ = 0x0;
  delete detector_;
  detector_ = 0x0;
  delete tracker_;
  tracker_ = 0x0;
  delete frame_;
  frame_ = 0x0;
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

void TTrack::CleanUp(){

  delete tracker_->GetPtrToFinishedFrame();

}

