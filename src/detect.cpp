#include "../headers/detect.hpp"
#include "../headers/helpers.hpp"
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

cv::Mat *Detect::GetPtrToClassifiedFrame() const {

  return frame_;

}

bool Detect::Found() const {

  return found_;

}

void Detect::Setup(const std::string &root_dir){

  root_dir_ = root_dir;

}

void Detect::LoadTrainingData(){

  // set up directories
  const std::string positive_im_dir(root_dir_ + "/data/positive_data/training_images/");
  const std::string positive_mask_dir(root_dir_ + "/data/positive_data/masks/");
  const std::string negative_im_dir(root_dir_ + "/data/negative_data/training_images/");
  
  //vectors to store the urls
  std::vector<std::string> positive_ims;
  std::vector<std::string> negative_ims;
  std::vector<std::string> positive_masks;
  size_t cols=0,rows=0;
  
  //get the file urls and the image sizes
  GetImageURLAndSize(positive_im_dir,positive_ims,cols,rows);
  GetImageURLAndSize(negative_im_dir,negative_ims,cols,rows);
  GetImageURLAndSize(positive_mask_dir,positive_masks,cols,rows);
  
  //preallocate training data storage
  training_data_ = cv::Mat(cv::Size(rows*cols,NDImage::channels_),CV_32FC1);
  training_labels_ = cv::Mat(cv::Size(rows,1),CV_32SC1);

  //load the data
  LoadPositive(positive_ims,positive_masks);
  LoadNegative(negative_ims);
  
}


void Detect::LoadPositive(const std::vector<std::string> &image_urls, const std::vector<std::string> &mask_urls){

  for(size_t im=0;im<image_urls.size();im++){
    std::cout << image_urls[im] << std::endl;
    cv::Mat image = cv::imread(image_urls[im]);
    cv::Mat mask = cv::imread(mask_urls[im]);
    
    try{
      nd_image = new NDImage(image);
      LoadPixels(nd_image,mask);
    }catch(std::runtime_error &e){
      std::cerr << "Error, image " << image_urls[im] << " failed to load.\n" << e.what() << "\n";
      exit(1); //cannot continue as training matrix will be messed up
    }catch(std::bad_alloc &e){
      std::cerr << "Error allocating memory for NDImage.\n" << e.what() << "\n";
      exit(1);
    }

    delete nd_image;
  
  }

}

void Detect::LoadNegative(const std::vector<std::string> &image_urls){
  
  //for each image LoadPixels
  
  for(size_t im=0;im<image_urls.size();im++){
    std::cout << image_urls[im] << std::endl;
    cv::Mat image = cv::imread(image_urls[im]);
    cv::Mat mask = cv::Mat::zeros(image.size(),image.type());
    
    try{
      nd_image = new NDImage(image);
      LoadPixels(nd_image,mask);
    }catch(std::runtime_error &e){
      std::cerr << "Error, loading " << image_urls[im] << " failed.\n" << e.what() << "\n";
      exit(1);
    }catch(std::bad_alloc &e){
      std::cerr << "Error allocating memeory of NDImage.\n" << e.what() << "\n";
      exit(1);
    }

    delete nd_image;
      
  }

}

void Detect::LoadPixels(const NDImage *nd_image, const cv::Mat &mask){

  if(nd_image->bad())
    throw std::runtime_error("Error, image data was corrupt/null");
  if(mask.data == 0)
    throw std::runtime_error("Error, mask data was corrupt/null");

  const int rows = nd_image->rows();
  const int cols = nd_image->cols();

  if(mask.rows != rows || mask.cols != cols)
    throw std::runtime_error("Error, image and mask have different dimensions");

  const unsigned char *mask_ptr = reinterpret_cast<unsigned char *>(mask.data);  
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      
      training_data_(cv::Range(r,r),cv::Range::all()) = nd_image->GetPixelData(r,c);
      const int index = (r * cols) + c;
      
      
      // if( mask_ptr[index] 
    }
  }

}

void Detect::Train(){

  // load training images images
  LoadTrainingData();

  // train


}
