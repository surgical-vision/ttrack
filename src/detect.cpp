#include "../headers/detect.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(){};

Detect::~Detect(){};

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


void Detect::Setup(const std::string &root_dir,const bool load_classifier){

  root_dir_ = root_dir;
  classifier_dir_ = root_dir + "/classifier/";

  if(load_classifier){
    try{
      classifier_.load((classifier_dir_ + "forest.xml").c_str());
    }catch(cv::Exception &e){
      std::cerr << e.what() << "\n";
      exit(1);
    }
  }  
}

void Detect::TrainSeparate(){
  
  train_ = new TrainSeparate();

  // load training images images
  train_->LoadTrainingData();

  //train the classifier
  train_->TrainClassifier();

}

void Detect::CrossValidate(const bool train, const int folds){
  
  train_ = new TrainCrossValidate();

  train->LoadCrossValidationData(folds);

  train_->TrainClassifier();

  TestCrossValidationData(folds);
  

}


void Detect::DebugTest(const std::string &infile, const std::string &outfile){
  
  cv::Mat in = cv::imread(infile),out;
  DebugTest(in,out);
  const std::string save_dir(root_dir_ + "/output/");
  boost::filesystem::create_directory(boost::filesystem::path(save_dir));
  cv::imwrite(save_dir + outfile,out);

}


void Detect::DebugTest(const cv::Mat &in_, cv::Mat &out){
  
  NDImage in(in_);

  out = cv::Mat(in_.size(),CV_8UC3);
  const int rows = in_.rows;
  const int cols = in_.cols;

  for(int r=0;r<rows;r++)
    for(int c=0;c<cols;c++)
      out.at<cv::Vec3b>(r,c) = cv::Vec3b(255*classifier_.predict_prob(in.GetPixelData(r,c)),0,0);
  
  
}


void Detect::LoadCrossValidationData(const int folds){

  const float training_fraction = 1 - 1.0/folds;

  
  







}
