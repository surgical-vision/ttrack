#include "../headers/detect.hpp"
#include "../headers/helpers.hpp"
#include "../headers/randomforest.hpp"
#include "../headers/supportvectormachine.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(boost::shared_ptr<std::string> root_dir, ClassifierType classifier_type, TrainType train_type):root_dir_(root_dir){

  //create a new classifier
  SetupClassifier(classifier_type);
  
  //will train the classifier and save if required. If not then load. 
  switch(train_type){
    case X_VALIDATE: TrainCrossValidate(10); break;
    case SEPARATE: TrainSeparate(); break;
    case NA: LoadClassifier(); break;
    default: throw(std::runtime_error("Unhandled train_type value. Fix this!"));
  }

  if(!Loaded()) throw(std::runtime_error("Error, could not construct classifier.\n"));

}

Detect::~Detect(){
  delete classifier_;
  classifier_ = 0x0;
}

void Detect::operator()(cv::Mat *image){
  

  if(image == 0) { //if the image is null then we must be at the last frame
    found_ = false; 
    return;
  }

  frame_ = image;
  
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
    exit(1);
  }

#if defined (DEBUG) || defined(_DEBUG_)
  assert(classifier_); //check it actually points to something now
#endif
} 

void Detect::LoadClassifier(){
  classifier_->Load( classifier_dir() + "/" + classifier_->NameAsString() + ".xml");
}

void Detect::TrainSeparate(){

  //construct training system and load the training data
  train_ = new TrainData(*root_dir_);
  train_->LoadSeparateTrainingData();
    
  //train the classifier
  classifier_->TrainClassifier(train_->training_data(),train_->training_labels(),*root_dir_);

  delete train_;
}

void Detect::TrainCrossValidate(const int folds){
  
  //construct training system and load the training data
  train_ = new TrainData(*root_dir_);
  train_->LoadCrossValidationData();
  
  //iterate over the folds
  for(int n=0;n<folds;n++){

    // load each part of the training data matrix into a submatrix
    boost::shared_ptr<cv::Mat> train_fmat,label_fmat,test_fmat,truth_fmat;
    train_->GetFoldMatrices(train_fmat,label_fmat,test_fmat,truth_fmat,n,folds);
    classifier_->TrainClassifier(train_fmat,label_fmat,*root_dir_);

    // get error values
    

  }
  
  delete train_;
 
}
