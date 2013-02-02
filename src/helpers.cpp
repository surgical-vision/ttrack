#include "../headers/headers.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>
#include "../headers/exceptions.hpp"


cv::Mat &ttrk::ConvertMatSingleToTriple(cv::Mat &im){

  //must return im!

  if(im.type() == CV_8UC3) return im;

  if(im.type() != CV_8UC1) throw(std::runtime_error("Error, unknown mask format. Expect 3 channel byte or single channel byte!\n"));

  cv::Mat tmp(im.size(),CV_8UC3);
  const int rows = im.rows;
  const int cols = im.cols;
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      uint8_t pix = im.at<uint8_t>(r,c);
      tmp.at<cv::Vec3b>(r,c) = cv::Vec3b(pix,pix,pix);
    }
  }

  im = tmp;
  return im;
}
