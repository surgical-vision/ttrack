#include "../headers/detect.hpp"
#include "../headers/helpers.hpp"
#include "../headers/randomforest.hpp"
#include "../headers/supportvectormachine.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(boost::shared_ptr<std::string> root_dir, TrainType train_type, ClassifierType classifier_type):root_dir_(root_dir){

  //create a new classifier
  SetupClassifier(classifier_type);
  
  //will train the classifier and save
  if(train_type == X_VALIDATE)
    TrainCrossValidate(10);
  
  else if(train_type == SEPARATE)
    TrainSeparate();
  
  else
    throw(std::runtime_error("Unhandled train_type value. Fix this!"));
  
}

Detect::Detect(boost::shared_ptr<std::string> root_dir, ClassifierType classifier_type):root_dir_(root_dir){

  LoadClassifier(classifier_type);
   
}

Detect::~Detect(){}

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

  assert(classifier_); //check it actually points to something now
  
}

void Detect::LoadClassifier(const ClassifierType type){
  
  std::string classifier_name;

  switch(type){
    
  case RF: classifier_name = "forest.xml"; break;
  case SVM: classifier_name = "svm.xml"; break;
  case NBAYES: classifier_name = "nbayes.xml"; break;
  default: classifier_name = "forest.xml"; break;

  }
  

  // use the specified classifier type to load the desired classifier
  try{
    
    classifier_->Load( classifier_dir() + classifier_name );

  }catch(cv::Exception &e){ //fix this to use a different exception class

    std::cerr << e.what() << "\n";
    exit(1);

  }

}

void Detect::TrainSeparate(){
  
  train_ = new TrainData(*root_dir_);
  // load training images images
  train_->LoadTrainingData(false);
    
  //train the classifier
  classifier_->TrainClassifier(*(train_->training_data()),*(train_->training_labels()),*root_dir_);

  delete train_;
}

void Detect::TrainCrossValidate(const int folds){
  
  train_ = new TrainData(*root_dir_);
  train_->LoadTrainingData(true);

  for(int n=0;n<folds;n++){

    // load each part of the training data matrix into a submatrix
    cv::Mat train_fmat;
    cv::Mat label_fmat;

    classifier_->TrainClassifier(train_fmat,label_fmat,*root_dir_);

    // get error values


  }
  
  delete train_;
 
}

/*void Detect::DebugTest(const std::string &infile, const std::string &outfile){
  
  cv::Mat in = cv::imread(infile),out;
  DebugTest(in,out);
  const std::string save_dir(root_dir_ + "/output/");
  boost::filesystem::create_directory(boost::filesystem::path(save_dir));
  cv::imwrite(save_dir + outfile,out);

  }*/

/*
void Detect::DebugTest(const cv::Mat &in_, cv::Mat &out){
  
  NDImage in(in_);

  out = cv::Mat(in_.size(),CV_8UC3);
  const int rows = in_.rows;
  const int cols = in_.cols;

  for(int r=0;r<rows;r++)
    for(int c=0;c<cols;c++)
      out.at<cv::Vec3b>(r,c) = cv::Vec3b(255*classifier_.predict_prob(in.GetPixelData(r,c)),0,0);
  
  
}
*/
/*
void Detect::LoadCrossValidationData(const int folds){

  const float training_fraction = 1 - 1.0/folds;



}
*/
