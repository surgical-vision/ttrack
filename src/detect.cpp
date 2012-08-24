#include "../headers/detect.hpp"
#include<iostream>

using namespace ttrk;

Detect::Detect():c_frame_(new cv::Mat){};

Detect::~Detect(){};


void Detect::operator()(boost::shared_ptr<cv::Mat> image){

  *c_frame_ = image->clone();
  
}

boost::shared_ptr<cv::Mat> Detect::GetPtrToClassified() const {

  return c_frame_;

}

bool Detect::Found() const {

  return true;

}
