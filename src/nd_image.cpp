#include "../headers/nd_image.hpp"

using namespace ttrk;

int NDImage::channels_ = _BGR_ + _HS_ + _O2_ + _O3_ + _XYZ_;

NDImage::NDImage(const cv::Mat &image){

   SetUpImages(image);

}


void NDImage::SetUpImages(const cv::Mat &image){


  rows_ = image.rows;
  cols_ = image.cols;
  const cv::Mat &bgr = image;

  if(_BGR_){
    cv::Mat red,green,blue;
    std::vector<cv::Mat>to;
    cv::split(bgr,to);
    images_.insert( NamedImage("blue",to[0]) );
    images_.insert( NamedImage("green",to[1]) );
    images_.insert( NamedImage("red",to[2]) );
  }
  if(_HS_){
    cv::Mat hue,sat;
    ConvertBGR2HS(bgr,hue,sat);
    images_.insert( NamedImage("hue",hue) );
    images_.insert( NamedImage("sat",sat) );
  }
  if(_O2_){
    cv::Mat o2;
    ConvertBGR2O2(bgr,o2);
    images_.insert( NamedImage("o2",o2) );
  }
  if(_O3_){
    cv::Mat o3;
    ConvertBGR2O3(bgr,o3);
    images_.insert( NamedImage("o3",o3) );
  }
  if(_XYZ_){
    cv::Mat xyz;
    cvtColor(bgr,xyz,CV_BGR2XYZ);
    std::vector<cv::Mat> to;
    cv::split(xyz,to);
    images_.insert(NamedImage("x",to[0]));
    images_.insert(NamedImage("y",to[1]));    
    images_.insert(NamedImage("z",to[2]));
  }

  bad_ = false;

}

/**
 * THIS FUNCTION IS BAD. MAKE IT LESS BAD.
 */

cv::Mat NDImage::GetPixelData(const int r, const int c) const {

  //set up the return vector, should be float type for the classifier
  cv::Mat return_pix(1,NDImage::channels_,CV_32FC1);

  //iterate over the images, get the pixel data
  std::map<std::string,cv::Mat>::const_iterator from;
  cv::MatIterator_<float> to;


  // Iterates in alphnumerical order. 
  
  for(from = images_.begin(),to = return_pix.begin<float>();from!=images_.end() && to!=return_pix.end<float>();from++,to++){

    //get a reference to the image
    const cv::Mat &im = from->second;

#ifdef DEBUG
    if(im.channels() != 1) std::cout << from->first << std::endl;
    assert(im.channels() == 1);
#endif

    // store the pixel in the return matrix, taking into account type
    if(im.depth() == CV_32F){
      *to = im.at<float>(r,c);
    }else if(im.depth() == CV_8U){
      *to = static_cast<float>(im.at<unsigned char>(r,c));
    }else{
      assert(0);
    }

  }

  return return_pix;
}


void NDImage::ConvertBGR2HS(const cv::Mat &in,cv::Mat &hue, cv::Mat &sat){
  
  cv::Mat huesat;
  cvtColor(in,huesat,CV_BGR2HSV);
  hue = cv::Mat(in.size(),CV_8UC1);
  sat = cv::Mat(in.size(),CV_8UC1);
  const int rows = in.rows;
  const int cols = in.cols;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const cv::Vec3b &hsv = huesat.at<cv::Vec3b>(r,c);
      hue.at<unsigned char>(r,c) = hsv[0];
      sat.at<unsigned char>(r,c) = hsv[1];
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
      out.at<float>(r,c) = 0.5f * (bgr[2] - bgr[1]);
     }
  }

}

void NDImage::ConvertBGR2O3(const cv::Mat &in,cv::Mat &out){

  const int rows = in.rows;
  const int cols = in.cols;
  out = cv::Mat(in.size(),CV_32FC1);
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      const cv::Vec3b &bgr = in.at<cv::Vec3b>(r,c);
      out.at<float>(r,c) = (0.5f*bgr[0]) - (0.25f*(bgr[2]+bgr[1]));
    }
  }

}

