#include <boost/ref.hpp>
#include <vector>
#include <cassert>
#include <string>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/system/error_code.hpp>

#include "../include/headers.hpp"
#include "../include/ttrack_app.hpp"
#include "../include/ttrack.hpp"
#include "../include/utils/helpers.hpp"
#include "../include/track/stt/stereo_tool_tracker.hpp"
#include "../include/track/stt/monocular_tool_tracker.hpp"

using namespace ttrk;

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const std::string &left_media_file,const std::string &right_media_file){
  
  SetUp(model_parameter_file,camera_calibration_file,classifier_path,results_dir,classifier_type,CameraType::STEREO);
  tracker_->Tracking(false); 
  handler_.reset(new StereoVideoHandler(left_media_file,right_media_file, results_dir_ + "/tracked_video.avi"));

}

void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const std::string &media_file){

  SetUp(model_parameter_file, camera_calibration_file, classifier_path, results_dir, classifier_type, CameraType::MONOCULAR);
  tracker_->Tracking(false); 
  if(IS_VIDEO(boost::filesystem::path(media_file).extension().string()))
    handler_.reset(new VideoHandler(media_file, results_dir_ + "/tracked_video.avi"));
  else if(boost::filesystem::is_directory(boost::filesystem::path(media_file)))
    handler_.reset(new ImageHandler(media_file, results_dir_ + "/tracked_frames/"));
  else
    throw(boost::filesystem::filesystem_error("Error, wrong file type\n",boost::system::error_code()));  

}


void TTrack::SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const CameraType camera_type){
  
  camera_type_ = camera_type;
  results_dir_ = results_dir;

  try{
  
    //if train type is NA, training is skipped
    detector_.reset(new Detect(classifier_path,classifier_type));
    
    //load the correct type of tool tracker
    switch(camera_type_){
    case STEREO:
      tracker_.reset(new StereoToolTracker(model_parameter_file,camera_calibration_file));
      break;
    case MONOCULAR:
      tracker_.reset(new MonocularToolTracker(model_parameter_file,camera_calibration_file));
      break;
    default:
      tracker_.reset(new StereoToolTracker(model_parameter_file,camera_calibration_file));
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

void TTrack::GetUpdate(std::vector<boost::shared_ptr<Model> > &models){

  detector_->Run(GetPtrToNewFrame());
  tracker_->Run(GetPtrToClassifiedFrame(), detector_->Found());

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

ClassifierType TTrack::ClassifierFromString(const std::string &classifier_name){

  if (classifier_name == "RF" || classifier_name == "rf"){
    return ClassifierType::RF;
  }
  else if (classifier_name == "SVM" || classifier_name == "svm"){
    return ClassifierType::SVM;
  }
  else if (classifier_name == "NB" || classifier_name == "nb"){
    return ClassifierType::NBAYES;
  }
  else{
    throw std::runtime_error("Error, bad classifier type");
  }

}

void TTrack::SaveFrame(){

  boost::shared_ptr<sv::Frame> frame = tracker_->GetPtrToFinishedFrame();

  //request the handler to save it to a video/image
  handler_->SaveFrame(frame->GetImageROI());

}

void TTrack::SaveResults() const {
  
  //boost::shared_ptr<const sv::Frame> frame = tracker_->GetPtrToFinishedFrame();
  //std::vector<KalmanTracker> &tracked_models = tracker_->TrackedModels();

  //for( size_t i = 0 ; i < tracked_models.size() ; i++ ){

  //  KalmanTracker &model = tracked_models[i];
  //  
  //  boost::shared_ptr<std::ofstream> results_file = model.SaveFile();

  //  if( !results_file->is_open() ){
  //    
  //    std::stringstream ss; ss << results_dir_ + "/model_pose" << i << ".txt";
  //    results_file->open( ss.str(),  std::ofstream::out);
  //    
  //  }

  //  cv::Vec3d angle_axis = model.CurrentPose().rotation_.AngleAxis();
  //  cv::Vec3d translation = model.CurrentPose().translation_;
  //  cv::Vec3d point_of_interest = model.CurrentPose().Transform(model.PtrToModel()->GetTrackedPoint());

  //  *results_file << translation[0] << "," << translation[1] << "," << translation[2] << "," << angle_axis[0] << "," << angle_axis[1] << "," << angle_axis[2] << "," << point_of_interest[0] << "," << point_of_interest[1] << "," << point_of_interest[2] << "\n" ;
  //  results_file->flush();

  //}

}

boost::shared_ptr<const sv::Frame> TTrack::GetPtrToCurrentFrame() const {
  return frame_;
}

boost::shared_ptr<sv::Frame> TTrack::GetPtrToNewFrame(){
  
  static cv::Mat stest = handler_->GetNewFrame();
    
  cv::Mat test = stest.clone();

  //if the input data has run out test will be empty, if this is so
  //reset the frame_ pointer to empty and return it. this will signal to 
  //the tracking/detect loop to stop
  if (test.data == 0x0) {
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


