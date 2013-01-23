#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
#include "../headers/surgical_tool_tracker.hpp"

using namespace ttrk;

void TTrack::SetUp(std::string root_dir, const ClassifierType classifier_type, const TrainType train_type){
  
  if(!boost::filesystem::exists(boost::filesystem::path(root_dir)))
    throw std::runtime_error("Error, directory " + root_dir + " does not exist.\n");
  
  try{
  
    //set the shared_ptr containing the directory where data is
    root_dir_.reset(new std::string(root_dir)); 

    //if train type is NA, training is skipped
    detector_.reset(new Detect(root_dir_,classifier_type,train_type));
    tracker_.reset(new SurgicalToolTracker(10,10));

  }catch(std::bad_alloc &e){
    std::cerr << "Error, memory error. Could not construct detector/tracker.\n" << e.what();
    std::cerr << "Exiting...\n";
    SAFE_EXIT();
  }catch(std::exception &e){
    std::cerr << "Error, caught exception.\n" <<  e.what();
    std::cerr << "Exiting...\n";
    SAFE_EXIT();
  }

  
}

void TTrack::Run(){

  (*detector_)( GetPtrToNewFrame() ); 

  while(detector_->Found()){
    
    boost::thread TrackThread(boost::ref(*(tracker_.get())), GetPtrToClassifiedFrame() );
    boost::thread DetectThread(boost::ref(*(detector_.get())), GetPtrToNewFrame() );

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
  handler_.reset(new VideoHandler(*root_dir_ + video_url, *root_dir_));
  Run();
 
}

void TTrack::RunImages(const std::string &image_url){

  tracker_->Tracking(false);
  handler_.reset(new ImageHandler(*root_dir_ + image_url, *root_dir_));
  Run();

}

void TTrack::SaveFrame(){

  //draws the model at the current pose on c_frame_
  boost::shared_ptr<cv::Mat> frame = tracker_->GetPtrToFinishedFrame();

  //DrawModel(frame_);
  
  handler_->SavePtrToFrame(frame);

}

void TTrack::SaveDebug() const {
  
  //iterate through all images, push to vector
  //send to iamge handler

}

void TTrack::DrawModel(boost::shared_ptr<cv::Mat> frame){

}

boost::shared_ptr<cv::Mat> TTrack::GetPtrToNewFrame(){
  
  frame_ = handler_->GetPtrToNewFrame(); //now point at new frame, discarding old
  return frame_;

}

boost::shared_ptr<cv::Mat> TTrack::GetPtrToClassifiedFrame(){

  frame_ = detector_->GetPtrToClassifiedFrame();
  return frame_;

}

/******** SETUP CODE ***********/

bool TTrack::constructed_ = false;

boost::scoped_ptr<TTrack> TTrack::instance_(new TTrack);

TTrack::TTrack(){}//:tracker_(0),detector_(0),handler_(0),frame_(0){}

TTrack::~TTrack(){
  /*delete handler_;
  handler_ = 0x0;
  delete detector_;
  detector_ = 0x0;
  delete tracker_;
  tracker_ = 0x0;
  delete frame_;
  frame_ = 0x0;*/
}

void TTrack::Destroy(){

  //delete instance_;
  instance_.reset();// = 0;
  constructed_ = false;

}

TTrack::TTrack(const TTrack &that){
  assert(0);
}

TTrack &TTrack::operator=(const TTrack &that){

  // check for self-assignment
  if(this == &that) return *this;
 
  //if we aren't self-assigning then something is wrong.
  throw(std::runtime_error("Error, attempting to construct a new TTrack.\n"));
  return *this;

}

TTrack &TTrack::Instance(){
  if(!constructed_){
    instance_.reset(new TTrack());
    constructed_ = true;
  }
  return *(instance_.get());
}

void TTrack::CleanUp(){

  //delete tracker_->GetPtrToFinishedFrame();

}

