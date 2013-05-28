#include "../headers/ttrack.hpp"
#include "../headers/utils/helpers.hpp"
#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
#include "../headers/track/stt/stereo_tool_tracker.hpp"
#include "../headers/track/stt/monocular_tool_tracker.hpp"

using namespace ttrk;

void TTrack::SetUp(std::string root_dir, const ClassifierType classifier_type, const CameraType camera_type){
  
  if(!boost::filesystem::exists(boost::filesystem::path(root_dir)))
    throw std::runtime_error("Error, directory " + root_dir + " does not exist.\n");
  
  camera_type_ = camera_type;

  try{
  
    //set the shared_ptr containing the directory where data is
    root_dir_.reset(new std::string(root_dir)); 

    //if train type is NA, training is skipped
    detector_.reset(new Detect(root_dir_,classifier_type));
    
    switch(camera_type_){
    case STEREO:
      tracker_.reset(new StereoToolTracker(3,20,*root_dir_ + "/config/camera.xml"));
      break;
    case MONOCULAR:
      tracker_.reset(new MonocularToolTracker(3,20,*root_dir_ + "/config/camera.xml"));
      break;
    default:
      tracker_.reset(new StereoToolTracker(3,20,*root_dir_ + "/config/camera.xml"));
      break;
    }

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

void TTrack::RunVideo(const std::string &video_url){
  
  //tracker_->Tracking(true);
  tracker_->Tracking(false); // SET this IN THE CONSTRUCTOR
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
  boost::shared_ptr<sv::Frame> frame = tracker_->GetPtrToFinishedFrame();

  //DrawModel(frame_);
  
  handler_->SavePtrToFrame(frame->PtrToMat());

}

void TTrack::SaveDebug() const {
  
  //iterate through all images, push to vector
  //send to iamge handler

}

void TTrack::DrawModel(cv::Mat &frame){

}

boost::shared_ptr<sv::Frame> TTrack::GetPtrToNewFrame(){
  
  boost::shared_ptr<cv::Mat> test = handler_->GetPtrToNewFrame();
  
  //if the input data has run out test will be empty, if this is so
  //reset the frame_ pointer to empty and return it. this will signal to 
  //the tracking/detect loop to stop
  if (!test) {
    frame_.reset();
    return frame_;
  }
    

  switch(camera_type_){
  
  case STEREO:
    frame_.reset( new sv::StereoFrame(test) );
    break;
  case MONOCULAR:
    frame_.reset( new sv::MonoFrame(test) );
    break;

  }
  
  return frame_;

}

boost::shared_ptr<sv::Frame> TTrack::GetPtrToClassifiedFrame(){

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

