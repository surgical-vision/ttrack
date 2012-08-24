#include "../headers/ttrack.hpp"
#include "../headers/helpers.hpp"
#include <vector>
#include <cassert>
#include <boost/filesystem.hpp>

using namespace ttrk;

void TTrack::Run(){

  // grab the first frame either from video or from image file
  v_frame_ = GetPtrToNewFrame();
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
    v_frame_ = GetPtrToNewFrame();
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

void TTrack::RunVideo(const std::string in_videofile, const std::string out_videofile,
                      const std::string out_posefile){
  mode_ = VIDEO;
  
  // open the capture to read from the videofile
  capture_.open(in_videofile);
  if(!capture_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + in_videofile + "\nPlease enter a new filename.\n");
    return;
  }

  // open the writer to create the processed video
  writer_.open(out_videofile,CV_FOURCC('M','J','P','G'), 25, cv::Size(capture_.get(CV_CAP_PROP_FRAME_HEIGHT),capture_.get(CV_CAP_PROP_FRAME_WIDTH)));
  if(!writer_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + out_videofile + " for saving.\nPlease enter a new filename.\n");
    return;
  }
  
  // store the file names (not really necessary right now) and run
  SetInputFile(in_videofile);
  SetOutputFile(out_videofile);
  Run();

}


void TTrack::RunImages(const std::string in_images, const std::string out_images,
                       const std::string out_posefile){

  using namespace boost::filesystem;
  mode_ = IMAGE;  

  // create a directory object
  path in_dir(in_images);
  if(!is_directory(in_dir)){
    std::runtime_error("Error, " + in_images + " is not a valid directory.\n");
  }
  
  // create a vector to save the filenames in the directory
  std::vector<path> images;
  copy(directory_iterator(in_dir),directory_iterator(),back_inserter(images));
  
  if(images.size() == 0){
    throw std::runtime_error("Error, no image files found in directory: " + in_images + "\nPlease enter a new filename.\n");
    return;
  }

  for(size_t i=0;i<images.size();i++){
    
    if( IS_IMAGE(images[i].extension()) ){
     
      SetInputFile(images[i].string());
      SetOutputFile(images[i].string());
      Run();
    
    }

  }    

}

void TTrack::SetInputFile(const std::string &url){
  in_filename_ = url;
}

void TTrack::SetOutputFile(const std::string &url){
  out_filename_ = url;
}

void TTrack::SaveFrame(){

  //draws the model at the current pose on c_frame_
  DrawModel();
  
  //saves the frame
  if(mode_ == VIDEO || mode_ == CAM){
    writer_ << *(c_frame_.get());
    return;
  }
  
  // mode == IMAGE
  imwrite(out_filename_,*(c_frame_.get()));

}

void TTrack::SaveDebug() const {

  // for each image in the detect_ and tracker_ classes, save their images

}


void TTrack::DrawModel(){


}

boost::shared_ptr<cv::Mat> TTrack::GetPtrToNewFrame(){
  
  if(mode_ == VIDEO || mode_ == CAM){
    boost::shared_ptr<cv::Mat> m;
    capture_ >> *m;
    return m;
  }
  
  // mode == IMAGE
  if(v_frame_->data == 0){
    // read the image and return it
    *v_frame_ = cv::imread(in_filename_);
    return v_frame_;
  }else{
    v_frame_.reset(); //reduce refcount of v_frame_ by one (to zero)
#ifdef DEBUG
    assert(v_frame_.use_count() == 0);
#endif
    return v_frame_;
  }

}

/******** SETUP CODE ***********/


bool TTrack::constructed_ = false;
TTrack *TTrack::instance_ = 0;

TTrack::TTrack():c_frame_(new cv::Mat),v_frame_(new cv::Mat){}
TTrack::~TTrack(){}

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


