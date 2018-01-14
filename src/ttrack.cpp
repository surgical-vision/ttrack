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

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, const ClassifierType classifier_type, const std::string &left_media_file, const std::string &right_media_file, const std::vector< std::vector<float> > &starting_poses, const size_t number_of_labels, const size_t skip_frames){
  
  SetUp(model_parameter_file, camera_calibration_file, classifier_path, results_dir, localizer_type, classifier_type, CameraType::STEREO, starting_poses, number_of_labels);
  tracker_->Tracking(false); 
  handler_.reset(new StereoVideoHandler(left_media_file,right_media_file, results_dir_ + "/tracked_video.avi", skip_frames));

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

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, const ClassifierType classifier_type, const std::string &media_file, const std::vector< std::vector<float> >&starting_poses, const size_t number_of_labels, const size_t skip_frames){

  SetUp(model_parameter_file, camera_calibration_file, classifier_path, results_dir, localizer_type, classifier_type, CameraType::MONOCULAR, starting_poses, number_of_labels);
  tracker_->Tracking(false); 
  if(IS_VIDEO(boost::filesystem::path(media_file).extension().string()))
    handler_.reset(new VideoHandler(media_file, results_dir_ + "/tracked_video.avi", skip_frames));
  else if(boost::filesystem::is_directory(boost::filesystem::path(media_file)))
    handler_.reset(new ImageHandler(media_file, results_dir_ + "/tracked_frames/"));
  else
    throw(boost::filesystem::filesystem_error("Error, wrong file type\n",boost::system::error_code()));  

}

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const LocalizerType &localizer_type, ClassifierType classifier_type, const CameraType camera_type, const std::vector< std::vector<float> > &starting_poses, const size_t number_of_labels){
  
  camera_type_ = camera_type;
  results_dir_ = results_dir;
  
  //if train type is NA, training is skipped
  //detector_.reset(new Detect(classifier_path, classifier_type, number_of_labels));

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


  tracker_->SetDetectorType(classifier_type, number_of_labels);

}

void TTrack::GetUpdate(std::vector<boost::shared_ptr<Model> > &models, const bool force_new_frame){

  static int count = 0;

  if (tracker_->HasConverged() || force_new_frame || tracker_->IsFirstRun()){
  
    Detect::global_detector_image = cv::Mat();

    //detector_->Run(GetPtrToNewFrame());

    //tracker_->Run(GetPtrToClassifiedFrame(), detector_->Found());
    tracker_->Run(GetPtrToNewFrame(), true);

    count++;

  }
  else{
    tracker_->RunStep();
    localizer_image_ = tracker_->GetLocalizerProgressFrame();
  }


  tracker_->GetTrackedModels(models);

  //if (count > 1000){
  //  handler_.reset();

  //}

}

void TTrack::RunThreaded(){

  //detector_->Run( GetPtrToNewFrame() ); 
  //
  //while( !handler_->Done() ){ //signals done by reading an empty image file either from a video or a directory of images
  //  
  //  boost::thread TrackThread(boost::ref(*(tracker_.get())), GetPtrToClassifiedFrame() , detector_->Found() );
  //  boost::thread DetectThread(boost::ref(*(detector_.get())), GetPtrToNewFrame() );

  //  TrackThread.join();
  //  DetectThread.join();
 
  //}
  
}  

LocalizerType TTrack::LocalizerTypeFromString(const std::string &str){

  if (str == "PWP3D_SIFT" ) return LocalizerType::PWP3D_SIFT;
  else if (str == "PWP3D_LK") return LocalizerType::PWP3D_LK;
  else if (str == "CompLS_SIFT") return LocalizerType::ComponentLS_SIFT;
  else if (str == "CompLS_LK") return LocalizerType::ComponentLS_LK;
  else if (str == "LSForest") return LocalizerType::LevelSetForest;
  else if (str == "ArticulatedCompLS_GradientDescent") return LocalizerType::ArticulatedComponentLS_GradientDescent;
  else if (str == "ArticulatedCompLS_GradientDescent_FrameToFrameLK") return LocalizerType::ArticulatedComponentLS_GradientDescent_F2FLK;
  else if (str == "ArticulatedCompLS_Sampler") return LocalizerType::ArticulatedComponentLS_WithSamping;
  else if (str == "PWP3D") return LocalizerType::PWP3D;
  else if (str == "LK") return LocalizerType::LK;
  else if (str == "CompLS") return LocalizerType::ComponentLS;
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

  if (!boost::filesystem::exists(results_dir_))
    boost::filesystem::create_directories(results_dir_);

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
  
  if (!boost::filesystem::exists(results_dir_))
    boost::filesystem::create_directories(results_dir_);

  std::vector<boost::shared_ptr<Model> > models;
  tracker_->GetTrackedModels(models);

  for (size_t i = 0; i < models.size(); i++){

    std::stringstream model_ss;
    model_ss << "icl_data_inst_" << i << ".txt";

    if (!models[i]->icl_output_file.is_open()) models[i]->icl_output_file.open(results_dir_ + "/" + model_ss.str());

    models[i]->WriteICLData(Localizer::occlusion_image);

    models[i]->WritePoseToFile();

    auto &model_debug_info = models[i]->debug_info;
    if (model_debug_info.tracked_feature_points.empty()) {
      ci::app::console() << "Nothing in frame." << std::endl;
      continue;
    }

    if (!model_debug_info.tracked_feature_points_writer.isOpened()){
      ci::app::console() << "Opening file" << std::endl;
      std::stringstream ss;
      ss << "feature_points_debug_model_" << i << ".avi";
      model_debug_info.tracked_feature_points_writer.open(results_dir_ + "/" + ss.str(), CV_FOURCC('M', 'J', 'P', 'G'), 25, model_debug_info.tracked_feature_points.size());
    }


    ci::app::console() << "Writing to frame..." << std::endl;
    model_debug_info.tracked_feature_points_writer << model_debug_info.tracked_feature_points;

  }

  cv::Mat d = GetCurrentDetectorImage();

  static cv::VideoWriter writer(results_dir_ + "/detector_output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, d.size());

  if (writer.isOpened()){
    writer << d;
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

//boost::shared_ptr<sv::Frame> TTrack::GetPtrToClassifiedFrame(){
//
//  frame_ = detector_->GetPtrToClassifiedFrame();
//
//  if (frame_ != nullptr){
//
//    cv::Mat m_channel = frame_->GetClassificationMap().clone();
//    std::vector<cv::Mat> channels;
//    for (size_t i = 0; i < frame_->NumClassificationChannels() && i < 3; ++i){
//
//      cv::Mat chan = sv::Frame::GetChannel(m_channel, i);
//      chan = 255 * chan;
//      chan.convertTo(chan, CV_8U);
//      
//
//      channels.push_back(chan);
//
//    }
//
//    cv::merge(channels, detector_image_);
//  }
//
//  detector_->ResetHandleToFrame(); //once classified detector has nothing to do with the old frame so it can forget about it
//  return frame_;
//
//}

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


