#include "../headers/ttrack.hpp"
#include <boost/program_options.hpp> 
#include "../headers/utils/helpers.hpp"

#if defined _WIN32 || _WIN64
  #include<direct.h>
#elif defined __linux__
#endif

/*
int main(int argc, char **argv){

  using namespace boost::program_options;

  bool stereo;
  std::string video_file,left_video_file,right_video_file,image_dir,classifier_type_,classifier_file,camera_calibration_file,model_file,results_dir;
  options_description desc("Options"); 
  desc.add_options() 
    ("help", "Print help messages")
    ("classifier_type",value<std::string>(&classifier_type_),"Type of classifier to load. Supported options: RF, SVM.")
    ("classifier_file", value<std::string>(&classifier_file), "Path to the classifier to load.")
    ("stereo", value<bool>( &stereo )->zero_tokens()  , "Flag to switch stereo on.")
    ("video_file",value<std::string>(&video_file),"path to monocular or stereo video file to do pose estimation/tracking on.")
    ("left_video_file",value<std::string>(&left_video_file),"path to left stereo video file to do pose estimation/tracking on.")
    ("right_video_file",value<std::string>(&right_video_file),"path to right stereo video file to do pose estimation/tracking on.")
    ("image_dir",value<std::string>(&image_dir),"path to directory containing images to do pose estimation/tracking on.")
    ("camera_calibration_file",value<std::string>(&camera_calibration_file),"path to directory containing images to do pose estimation/tracking on.")
    ("model_file",value<std::string>(&model_file),"path to file where model calibration is stored.")
    ("results_dir",value<std::string>(&results_dir),"path to directory where images can be saved.")   
    ; 

  variables_map vm; 
  ttrk::CameraType camera_type;
  ttrk::ClassifierType classifier_type;

  try{ 

    store(parse_command_line(argc, argv, desc), vm); // can throw 

    if(vm.count("help")){ 
      std::cout << "Basic Command Line Parameter App" << std::endl << desc << std::endl; 
      ttrk::SAFE_EXIT(0);
    } 

    notify(vm); // throws on error, so do after help in case 

    if(stereo) camera_type = ttrk::STEREO;
    else camera_type = ttrk::MONOCULAR;

    if(classifier_type_ == "RF" || classifier_type_ == "rf")
      classifier_type = ttrk::RF;
    else if (classifier_type_ == "SVM" || classifier_type_ == "svm")
      classifier_type = ttrk::SVM;
    else
      throw(boost::program_options::error("Incorrect classifier type given. Options are: rf, svm."));

    // there are any problems 
  }catch(error& e){ 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cerr << desc << std::endl; 
    ttrk::SAFE_EXIT(1); 
  } 

  
  ttrk::TTrack &t = ttrk::TTrack::Instance();

  try{
    
#if defined(_WIN32) || defined(_WIN64)
     //_chdir("../");
#endif



     t.SetUp(model_file,camera_calibration_file,classifier_file,results_dir, classifier_type, camera_type);
     

     if(video_file.length())
      t.RunVideo(video_file);
     else if (left_video_file.length() && right_video_file.length())
       t.RunVideo(left_video_file,right_video_file);
     else if (image_dir.length())
      t.RunImages(image_dir);
     else
       throw(std::runtime_error("Error, cmd line args\n"));

  }catch(std::runtime_error &e){

    std::cerr << e.what() << "\n";
    ttrk::SAFE_EXIT(1);
  
  }catch(std::exception &e){
  
    std::cerr << e.what() << "\n";
    ttrk::SAFE_EXIT(1);
  
  }
  
  
  ttrk::TTrack::Destroy();

#if defined(_WIN32) || defined(_WIN64)
  _CrtDumpMemoryLeaks();
#endif
  
  ttrk::SAFE_EXIT(0);

}*/



