#include "../headers/nd_image.hpp"

using namespace ttrk;

int NDImage::channels_ = _RGB_ + _HS_ + _O2_ + _O3_ ;

NDImage::NDImage(const cv::Mat &image){

  images.insert( NamedImage("rgb",image) );
  
  SetUpImages();

}


void NDImage::SetUpImages(){

  cv::Mat &rgb = images["rgb"];
  if(_HS_){
    cv::Mat hs;
    //ConvertRGB2HS(rgb,hs);
    images.insert( NamedImage("hs",hs) );
  }
  if(_O2_){
    cv::Mat o2;
    //ConvertRGB2O2(rgb,o2);
    images.insert( NamedImage("o2",o2) );
  }
  if(_O3_){
    cv::Mat o3;
    //ConvertRGB2O3(rgb,o3);
    images.insert( NamedImage("o3",o3) );
  }

}

cv::Mat NDImage::GetPixelData(const int r, const int c) const {



}
