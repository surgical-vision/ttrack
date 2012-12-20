#include "../headers/train.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>

using namespace ttrk;

void Train::TestClassifier(BaseClassifier &to_train, boost::shared_ptr<cv::Mat> train, boost::shared_ptr<cv::Mat> truth){
  //for sample in test 
  //compare results to truth

}

void Train::LoadImages(const std::vector<std::string> &image_urls, const std::vector<std::string> &mask_urls, const LoadType type){

  //for each image
  for(size_t im=0;im<image_urls.size();im++){

    //load the image and the mask
    cv::Mat image = cv::imread(image_urls[im]);
    cv::Mat mask;

    if(type == POSITIVE || type == BOTH)
      mask = cv::imread(mask_urls[im]);
    else //type == NEGATIVE
      mask = cv::Mat::zeros(image.size(),image.type());

	//create the ndimage and load its pixels to the training data mask
    NDImage *nd_image_;
    try{

      nd_image_ = new NDImage(image);
      //LoadPixels(nd_image_,mask,type);

    }catch(std::runtime_error &e){
      std::cerr << "Error, image " << image_urls[im] << " failed to load.\n" << e.what() << "\n";
      exit(1); //cannot continue as training matrix will be messed up
    }catch(std::bad_alloc &e){
      std::cerr << "Error allocating memory for NDImage.\n" << e.what() << "\n";
      exit(1);
    }

    delete nd_image_;
  
  }

}

void LoadPixels(const NDImage *nd_image, const cv::Mat &mask, const LoadType type){
  int x=0;
  x++;
}

Train::Train(boost::shared_ptr<std::string> root_dir):root_dir_(root_dir){}

Train::Train(){}

Train::~Train(){
   
}

