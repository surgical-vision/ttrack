#include "../headers/detect.hpp"
#include<iostream>
#include <stdlib.h>
#include <time.h>

using namespace ttrk;

Detect::Detect(){};

Detect::~Detect(){};

void Detect::operator()(boost::shared_ptr<cv::Mat> image){
  

  if(image.get() == 0) {
    found_ = false;
    return;
  }
  v_frame_ = image;

  c_frame_.reset();

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
