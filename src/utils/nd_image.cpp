#include "../../include/ttrack/utils/nd_image.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace ttrk;

NDImage::NDImage(const cv::Mat &image){

   SetUpImages(image);

}


void NDImage::SetUpImages(const cv::Mat &image){

  rows_ = image.rows;
  cols_ = image.cols;

  images_new_ = cv::Mat(image.rows, image.cols, CV_32FC4);
  cv::Mat hue,sat;
  ConvertBGR2HS(image,hue,sat);
  images_.insert(NamedImage("hue", hue));
  images_.insert(NamedImage("sat", sat));
  cv::Mat o1;
  ConvertBGR2O1(image,o1);
  images_.insert( NamedImage("o1",o1) );
  cv::Mat o2;
  ConvertBGR2O2(image,o2);
  images_.insert( NamedImage("o2",o2) );

  float *image_data = (float *)images_new_.data;
  float *hue_data = (float *)hue.data;
  float *sat_data = (float *)sat.data;
  float *o1_data = (float *)o1.data;
  float *o2_data = (float *)o2.data;

  const int rows = image.rows;
  const int cols = image.cols;
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const int index = r*cols + c;
      const int index2 = index*4;
      image_data[index2] = hue_data[index];
      image_data[index2+1] = sat_data[index];
      image_data[index2+2] = o1_data[index];
      image_data[index2+3] = o2_data[index];
    }
  }
}

void NDImage::ConvertBGR2HS(const cv::Mat &in,cv::Mat &hue, cv::Mat &sat){
  
  cv::Mat huesat;
  cvtColor(in,huesat,CV_BGR2HSV);
  hue = cv::Mat(in.size(),CV_32FC1);
  sat = cv::Mat(in.size(),CV_32FC1);
  const int rows = in.rows;
  const int cols = in.cols;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const cv::Vec3b &hsv = huesat.at<cv::Vec3b>(r,c);
      hue.at<float>(r,c) = hsv[0];
      sat.at<float>(r,c) = hsv[1];
    }
  }
}

void NDImage::ConvertBGR2O1(const cv::Mat &in,cv::Mat &out){

  const int rows = in.rows;
  const int cols = in.cols;
  out = cv::Mat(in.size(),CV_32FC1);

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const cv::Vec3b &bgr = in.at<cv::Vec3b>(r,c);
      out.at<float>(r,c) = 0.5f * ((float)bgr[2] - (float)bgr[1]);
     }
  }

}

void NDImage::ConvertBGR2O2(const cv::Mat &in,cv::Mat &out){

  const int rows = in.rows;
  const int cols = in.cols;
  out = cv::Mat(in.size(),CV_32FC1);
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const cv::Vec3b &bgr = in.at<cv::Vec3b>(r,c);
      out.at<float>(r,c) = (0.5f*(float)bgr[0]) - (0.25f*((float)bgr[2]+(float)bgr[1]));
    }
  }

}

