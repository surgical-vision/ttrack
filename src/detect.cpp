#include "../headers/detect.hpp"
#include "../headers/helpers.hpp"
#include "../headers/randomforest.hpp"
#include "../headers/supportvectormachine.hpp"
#include "../headers/cross_validate.hpp"
#include "../headers/train_separate.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(boost::shared_ptr<std::string> root_dir, ClassifierType classifier_type, TrainType train_type):root_dir_(root_dir){

  //create a new classifier
  SetupClassifier(classifier_type);

  if(train_type == X_VALIDATE)
    CrossValidate c(root_dir,*classifier_,10);
  else if(train_type == SEPARATE)
    TrainSeparate s(root_dir,*classifier_);
  else if(train_type == NA)
    LoadClassifier();
  else
    assert(0);

  if(!Loaded()) throw(std::runtime_error("Error, could not construct classifier.\n"));

}

Detect::~Detect(){}

void Detect::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);
  ClassifyFrame();
  
}

void Detect::ClassifyFrame(){

#ifdef DEBUG 
  assert(Loaded());
  assert(frame_->type() == CV_8UC3);
#endif

  NDImage nd_image(*frame_);
  const int rows = frame_->rows;
  const int cols = frame_->cols;

  size_t DEBUG_COUNT = 0;
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      cv::Mat pix = nd_image.GetPixelData(r,c);
      
      size_t prediction = 255*(classifier_->PredictClass(nd_image.GetPixelData(r,c)));
      frame_->at<cv::Vec3b>(r,c) = cv::Vec3b((unsigned char)prediction,(unsigned char)prediction,(unsigned char)prediction);

      if(prediction > 0) DEBUG_COUNT++;

    }
  }

  if(DEBUG_COUNT>300) found_ = true;
  else found_ = false;

}

void Detect::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  //if the image is null then we must be at the last frame
  if(image == 0x0 || image->data == 0x0) { 
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

void Detect::LoadClassifier(){
  classifier_->Load( classifier_dir() + "/" + classifier_->NameAsString() + ".xml");
}
