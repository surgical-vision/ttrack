#include "../headers/detect.hpp"
#include<iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(){};

Detect::~Detect(){};

void Detect::operator()(boost::shared_ptr<cv::Mat> image){
  

  if(image.get() == 0) { //if the image is null then we must be at the last frame
    found_ = false; 
    return;
  }

  v_frame_ = image;

  c_frame_.reset(); //ditch reference to c_frame_ and classify a new image

  //do classification
  c_frame_ = boost::shared_ptr<cv::Mat>(new cv::Mat);

  *c_frame_ = v_frame_.get()->clone();

  found_ = true;
  
}

boost::shared_ptr<cv::Mat> Detect::GetPtrToClassified() const {

  return c_frame_;

}

bool Detect::Found() const {

  return found_;

}

void Detect::Setup(const std::string &root_dir){

  root_dir_ = root_dir;

}
