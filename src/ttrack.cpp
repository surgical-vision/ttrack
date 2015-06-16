#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/system/error_code.hpp>

#include "../include/ttrack/headers.hpp"
#include "../include/ttrack/ttrack_app.hpp"
#include "../include/ttrack/ttrack.hpp"
#include "../include/ttrack/utils/helpers.hpp"
#include "../include/ttrack/track/tracker/stereo_tool_tracker.hpp"
#include "../include/ttrack/track/tracker/monocular_tool_tracker.hpp"

using namespace ttrk;

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, const ClassifierType classifier_type, const std::string &left_media_file, const std::string &right_media_file, const std::vector< std::vector<float> > &starting_poses, const size_t number_of_labels){
  
  SetUp(model_parameter_file, camera_calibration_file, classifier_path, results_dir, localizer_type, classifier_type, CameraType::STEREO, starting_poses, number_of_labels);
  tracker_->Tracking(false); 
  handler_.reset(new StereoVideoHandler(left_media_file,right_media_file, results_dir_ + "/tracked_video.avi"));

}

std::vector<float> TTrack::PoseFromString(const std::string &pose_as_string){

  std::vector<float> string_as_floats;
  std::stringstream ss(pose_as_string);
  
  float x;
  while (ss >> x){
    string_as_floats.push_back(x);  
  }

  if (string_as_floats.size() != 15) throw std::runtime_error("Error, size is bad!");

  return string_as_floats;

}

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, const ClassifierType classifier_type, const std::string &media_file, const std::vector< std::vector<float> >&starting_poses, const size_t number_of_labels){

  SetUp(model_parameter_file, camera_calibration_file, classifier_path, results_dir, localizer_type, classifier_type, CameraType::MONOCULAR, starting_poses, number_of_labels);
  tracker_->Tracking(false); 
  if(IS_VIDEO(boost::filesystem::path(media_file).extension().string()))
    handler_.reset(new VideoHandler(media_file, results_dir_ + "/tracked_video.avi"));
  else if(boost::filesystem::is_directory(boost::filesystem::path(media_file)))
    handler_.reset(new ImageHandler(media_file, results_dir_ + "/tracked_frames/"));
  else
    throw(boost::filesystem::filesystem_error("Error, wrong file type\n",boost::system::error_code()));  

}

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, ClassifierType classifier_type, const CameraType camera_type, const std::vector< std::vector<float> > &starting_poses, const size_t number_of_labels){
  
  camera_type_ = camera_type;
  results_dir_ = results_dir;

  if (!boost::filesystem::exists(results_dir_))
    boost::filesystem::create_directories(results_dir_);
  
  //if train type is NA, training is skipped
  detector_.reset(new Detect(classifier_path, classifier_type, number_of_labels));

  //load the correct type of tool tracker
  switch (camera_type_){
  case STEREO:
    tracker_.reset(new StereoToolTracker(model_parameter_file, camera_calibration_file, results_dir_, localizer_type, number_of_labels));
    break;
  case MONOCULAR:
    tracker_.reset(new MonocularToolTracker(model_parameter_file, camera_calibration_file, results_dir_, localizer_type, number_of_labels));
    break;
  default:
    tracker_.reset(new StereoToolTracker(model_parameter_file, camera_calibration_file, results_dir_, localizer_type, number_of_labels));
    break;
  }


  for (auto &starting_pose : starting_poses)
    tracker_->AddStartPose(starting_pose);

}

void TTrack::GetUpdate(std::vector<boost::shared_ptr<Model> > &models, const bool force_new_frame){

  if (tracker_->HasConverged() || force_new_frame){
    
    detector_->Run(GetPtrToNewFrame());

    tracker_->Run(GetPtrToClassifiedFrame(), detector_->Found());

  }
  else{
    tracker_->RunStep();
  }

  localizer_image_ = tracker_->GetLocalizerProgressFrame();

  tracker_->GetTrackedModels(models);

}

void TTrack::RunThreaded(){

  detector_->Run( GetPtrToNewFrame() ); 
  
  while( !handler_->Done() ){ //signals done by reading an empty image file either from a video or a directory of images
    
    boost::thread TrackThread(boost::ref(*(tracker_.get())), GetPtrToClassifiedFrame() , detector_->Found() );
    boost::thread DetectThread(boost::ref(*(detector_.get())), GetPtrToNewFrame() );

    TrackThread.join();
    DetectThread.join();
 
  }
  
}  

LocalizerType TTrack::LocalizerTypeFromString(const std::string &str){

  if (str == "PWP3D" || str == "pwp3d") return LocalizerType::PWP3DLocalizer;
  else if (str == "CompLS") return LocalizerType::ComponentLS;
  else if (str == "ArticultatedComponentLS") return LocalizerType::ArticulatedComponentLS;
#ifdef USE_CERES
  else if (str == "CeresLevelSetSolver") return LocalizerType::CeresLevelSetSolver;
#endif
  else throw std::runtime_error("");

}

ClassifierType TTrack::ClassifierFromString(const std::string &classifier_name){

  std::string classifier_name_lower = classifier_name;
  std::transform(classifier_name.begin(), classifier_name.end(), classifier_name_lower.begin(), ::tolower);

  if (classifier_name_lower == "rf"){
    return ClassifierType::RF;
  }
  else if (classifier_name_lower == "mcrf"){
    return ClassifierType::MCRF;
  }
  else if (classifier_name_lower == "svm"){
    return ClassifierType::SVM;
  }
  else if (classifier_name_lower == "nb"){
    return ClassifierType::NBAYES;
  }
  else if (classifier_name_lower == "histogram"){
    return ClassifierType::HISTOGRAM;
  }
  else{
    throw std::runtime_error("Error, bad classifier type");
  }

}

void TTrack::SaveFrame(const cv::Mat &frame, bool flip) {

  cv::Mat f;
  if (flip){
    cv::flip(frame, f, 0);
  }
  else{
    f = frame.clone();
  }

  //request the handler to save it to a video/image
  handler_->SaveFrame(f);

}

void TTrack::SaveResults() {
  
  std::vector<boost::shared_ptr<Model> > models;
  tracker_->GetTrackedModels(models);

  for (size_t i = 0; i < models.size(); i++){

    models[i]->WritePoseToFile();

  }

}

boost::shared_ptr<const sv::Frame> TTrack::GetPtrToCurrentFrame() const {
  return frame_;
}

boost::shared_ptr<sv::Frame> TTrack::GetPtrToNewFrame(){
  
  cv::Mat frame = handler_->GetNewFrame();

  //if the input data has run out frame will be empty, if this is so
  //reset the frame_ pointer to empty and return it. this will signal to 
  //the tracking/detect loop to stop
  if (frame.data == 0x0) {
    frame_.reset();
    return frame_;
  }
    

  switch(camera_type_){
  
  case STEREO:
    frame_.reset(new sv::StereoFrame(frame));
    break;
  case MONOCULAR:
    frame_.reset(new sv::MonoFrame(frame));
    break;
  }
  
  return frame_;

}

boost::shared_ptr<sv::Frame> TTrack::GetPtrToClassifiedFrame(){

  frame_ = detector_->GetPtrToClassifiedFrame();

  cv::Mat m_channel = frame_->GetClassificationMap().clone();
  std::vector<cv::Mat> channels;
  for (size_t i = 0; i < frame_->NumClassificationChannels() && i < 3; ++i){

    cv::Mat chan = sv::Frame::GetChannel(m_channel, i);
    chan.convertTo(chan, CV_8U);
    chan = 255 * chan;

    channels.push_back(chan);
    
  }

  cv::merge(channels, detector_image_);
  
  detector_->ResetHandleToFrame(); //once classified detector has nothing to do with the old frame so it can forget about it
  return frame_;

}

/******** SETUP CODE ***********/

bool TTrack::constructed_ = false;

boost::scoped_ptr<TTrack> TTrack::instance_;

TTrack::TTrack(){}

TTrack::~TTrack(){}

void TTrack::Destroy(){

  instance_.reset();
  constructed_ = false;

}

TTrack::TTrack(const TTrack &that){}

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


