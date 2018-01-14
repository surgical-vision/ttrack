#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include "../../include/ttrack/detect/detect.hpp"
#include "../../include/ttrack/utils/helpers.hpp"
#include "../../include/ttrack/detect/randomforest.hpp"
#include "../../include/ttrack/detect/supportvectormachine.hpp"
#include "../../include/ttrack/detect/histogram.hpp"
#include "../../include/ttrack/detect/multiclass_randomforest.hpp"

#include <cinder/app/App.h>

using namespace ttrk;

cv::Mat Detect::global_detector_image;

Detect::Detect(const std::string &classifier_path, ClassifierType classifier_type, const size_t number_of_labels){

  //create a new classifier
  SetupClassifier(classifier_type, number_of_labels);

  //LoadClassifier(classifier_path);

  //if (classifier_->IsBinary() && number_of_labels != 2){
  //  throw std::runtime_error("Error, incompatible classifier setup.\n");
  //}

  //currently i don't know a good way of checking if the opencv ML classifier has loaded
  //if(!Loaded()) throw(std::runtime_error("Error, could not construct classifier.\n"));

  needs_retrain_ = true;

}

Detect::~Detect(){}

void Detect::operator()(boost::shared_ptr<sv::Frame> image, const cv::Mat &sdf){

  SetHandleToFrame(image);
  ClassifyFrame(sdf);

}

void Detect::Run(boost::shared_ptr<sv::Frame> image, const cv::Mat &sdf){

    SetHandleToFrame(image);
    ClassifyFrame(sdf);

}


void Detect::ClassifyFrame(const cv::Mat &sdf){

  //double t = (double)cv::getTickCount();

  found_ = classifier_->ClassifyFrame(frame_, sdf);

  //t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  //ci::app::console() << "Processing detect time = " << t << std::endl;
  
  cv::Mat m_channel = frame_->GetClassificationMap().clone();
  std::vector<cv::Mat> channels;
  for (size_t i = 0; i < frame_->NumClassificationChannels() && i < 3; ++i){

    cv::Mat chan = sv::Frame::GetChannel(m_channel, i);
    chan = 255 * chan;
    chan.convertTo(chan, CV_8U);


    channels.push_back(chan);

  }

  if (global_detector_image.empty()){
    global_detector_image = cv::Mat::zeros(frame_->GetImage().size(), CV_8UC3);
  }

  cv::merge(channels, global_detector_image);
}

void Detect::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){

  //if the image is null then we must be at the last frame
  if(image == 0x0) {
    found_ = false;
    return;
  }

  frame_.reset();
  //assign the detector's frame pointer to this image
  frame_ = image;

}

void Detect::ResetHandleToFrame(){

  frame_.reset();

}

void Detect::SetupClassifier(const ClassifierType type, const size_t number_of_labels){

  classifier_.reset();

  switch (type){

  case MCRF: classifier_ = boost::static_pointer_cast<BaseClassifier, MultiClassRandomForest>(boost::shared_ptr<MultiClassRandomForest>(new MultiClassRandomForest(number_of_labels))); break;
  case RF: classifier_ = boost::static_pointer_cast<BaseClassifier, RandomForest>(boost::shared_ptr<RandomForest>(new RandomForest)); break;
  case SVM: classifier_ = boost::static_pointer_cast<BaseClassifier, SupportVectorMachine>(boost::shared_ptr<SupportVectorMachine>(new SupportVectorMachine)); break;
  case NBAYES: throw("NBAYES not supported"); //NOT YET IMPLEMENTED!
  case HISTOGRAM: classifier_ = boost::static_pointer_cast<BaseClassifier, Histogram>(boost::shared_ptr<Histogram>(new Histogram)); break;
  default: classifier_ = boost::static_pointer_cast<BaseClassifier, RandomForest>(boost::shared_ptr<RandomForest>(new RandomForest)); break;
  
  }
}

void Detect::LoadClassifier(const std::string &classifier_path){
  if(!boost::filesystem::exists(classifier_path)) throw(std::runtime_error("Error, the classifier at: " + classifier_path + " does not exist. Exiting...\n"));
  classifier_->Load(classifier_path);
}
