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

  //will train the classifier and save if required. If not then load. 
  //switch(train_type){
    //case X_VALIDATE: CrossValidate *cv = new CrossValidate(root_dir,*classifier_,10); delete cv; break;
    //case SEPARATE: TrainSeparate *s = new TrainSeparate(root_dir,*classifier_); delete s; break;
  //case X_VALIDATE: CrossValidate cv(root_dir,*classifier_,10); break; 
    //case NA: LoadClassifier(); break;
    //default: throw(std::runtime_error("Unhandled train_type value. Fix this!"));
  //}

  if(!Loaded()) throw(std::runtime_error("Error, could not construct classifier.\n"));

}

Detect::~Detect(){
  delete classifier_;
  classifier_ = 0x0;
}

void Detect::operator()(cv::Mat *image){
  

  if(image == 0x0 || image->data == 0x0) { //if the image is null then we must be at the last frame
    found_ = false; 
    return;
  }

  frame_.reset(image);
  
  circle(*frame_,cv::Point(100,100),20,cv::Scalar(200,182,233),-1);
  
  //do classification
  

  found_ = true;
  
}

void Detect::SetupClassifier(const ClassifierType type){
  
  classifier_ = 0x0;

  //construct the classifier from scratch
  try{

    switch(type){

      case RF: classifier_ = new RandomForest; break;
      case SVM: classifier_ = new SupportVectorMachine; break;    
      case NBAYES: throw("NBAYES not supported"); //NOT YET IMPLEMENTED!
      default: classifier_ = new RandomForest; break;
      
    }

  }catch(std::bad_alloc &e){
    std::cerr << "Error, could not create classifier: " << e.what();
    SAFE_EXIT();
  }

#if defined (DEBUG) || defined(_DEBUG_)
  assert(classifier_); //check it actually points to something now
#endif
} 

void Detect::LoadClassifier(){
  classifier_->Load( classifier_dir() + "/" + classifier_->NameAsString() + ".xml");
}
