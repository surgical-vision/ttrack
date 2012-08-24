#include "../headers/detect.hpp"
#include<iostream>

using namespace ttrk;

Detect::Detect():v2c_frame_(new cv::Mat){};

Detect::~Detect(){};

void Detect::operator()(boost::shared_ptr<cv::Mat> image){
  
  if(image.get() == 0) {
    found_ = false;
    return;
  }

  found_ = true;
  v2c_frame_ = image;
  
}

boost::shared_ptr<cv::Mat> Detect::GetPtrToClassified() const {
  
  boost::shared_ptr<cv::Mat> m(new cv::Mat);
  *m = v2c_frame_->clone();
  return m;

}

bool Detect::Found() const {

  return found_;

}
