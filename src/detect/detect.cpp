#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include "../../include/detect/detect.hpp"
#include "../../include/utils/helpers.hpp"
#include "../../include/detect/randomforest.hpp"
#include "../../include/detect/supportvectormachine.hpp"


using namespace ttrk;

Detect::Detect(const std::string &classifier_path, ClassifierType classifier_type){

  //create a new classifier
  SetupClassifier(classifier_type);

  LoadClassifier(classifier_path);

  //currently i don't know a good way of checking if the opencv ML classifier has loaded
  //if(!Loaded()) throw(std::runtime_error("Error, could not construct classifier.\n"));

}

Detect::~Detect(){}

void Detect::operator()(boost::shared_ptr<sv::Frame> image){

  SetHandleToFrame(image);
  ClassifyFrame();

}

void Detect::Run(boost::shared_ptr<sv::Frame> image){

    SetHandleToFrame(image);
    ClassifyFrame();

}


void Detect::ClassifyFrame(){

  if (frame_ == nullptr) return;

  assert(Loaded());
  assert(frame_->GetImageROI().type() == CV_8UC3);

  cv::Mat whole_frame = frame_->GetImage();
  NDImage nd_image(whole_frame);
  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  size_t pixel_count = 0;

  //unsigned char *frame_data = (unsigned char *)frame_->PtrToClassificationMap()->data;
  float *frame_data = (float *)frame_->GetClassificationMap().data;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      const int index = r*cols + c;

      cv::Mat pix = nd_image.GetPixelData(r,c);

      float hue = pix.at<float>(0);
      float sat = pix.at<float>(1);
      float o1 = pix.at<float>(2);
      float o2 = pix.at<float>(3);

      //const unsigned char prediction = (unsigned char)255*classifier_->PredictClass(pix);
      const float prediction = (const float)classifier_->PredictProb(pix, 1); //need to be between 0 - 255 for later processing stage

      frame_data[index] = prediction;

      pixel_count += prediction > 0;

    }
  }

  if(pixel_count > (0.02*rows*cols)) found_ = true;
  else found_ = false;

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

void Detect::SetupClassifier(const ClassifierType type){

  classifier_.reset();

  //construct the classifier from scratch
  try{

    switch(type){

      case RF: classifier_ = boost::static_pointer_cast<BaseClassifier, RandomForest>(boost::shared_ptr<RandomForest>(new RandomForest)); break;
      case SVM: classifier_ = boost::static_pointer_cast<BaseClassifier, SupportVectorMachine>(boost::shared_ptr<SupportVectorMachine>(new SupportVectorMachine )); break;
      case NBAYES: throw("NBAYES not supported"); //NOT YET IMPLEMENTED!
      default: classifier_ = boost::static_pointer_cast<BaseClassifier, RandomForest>(boost::shared_ptr<RandomForest>(new RandomForest)); break;

    }

  }catch(std::bad_alloc &e){
    std::cerr << "Error, could not create classifier: " << e.what();
    SAFE_EXIT();
  }

#if defined (DEBUG) || defined(_DEBUG_)
  assert(classifier_.get()); //check it actually points to something now
#endif
}

void Detect::LoadClassifier(const std::string &classifier_path){
  if(!boost::filesystem::exists(classifier_path)) throw(std::runtime_error("Error, the classifier at: " + classifier_path + " does not exist. Exiting...\n"));
  classifier_->Load(classifier_path);
}
