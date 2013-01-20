#include "../headers/train_separate.hpp"
#include "../headers/helpers.hpp"
#include "../headers/im_mask_set.hpp"
#include "../headers/exceptions.hpp"

using namespace ttrk;

TrainSeparate::TrainSeparate(boost::shared_ptr<std::string> root_dir, BaseClassifier &to_train):Train(root_dir){

  //
  LoadTrainingData();

  to_train.TrainClassifier(training_data_,training_labels_,root_dir_);

}

void TrainSeparate::LoadTrainingData() {

  ImageMaskSet positive_set(ImageMaskSet::POSITIVE,*(root_dir_.get()));
  ImageMaskSet negative_set(ImageMaskSet::NEGATIVE,*(root_dir_.get()));
  ImageMaskSet normal_set(ImageMaskSet::BOTH,*(root_dir_.get()));

  //preallocate training data storage
  size_t all_pix = positive_set.NumPix() + negative_set.NumPix() + normal_set.NumPix();
  if(positive_set.NumPix() + normal_set.NumPix() <= 0){
    std::cerr << "No positive data found!\n";
    SAFE_EXIT();
  }

  training_data_ = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::Size(NDImage::channels_,(int)all_pix),CV_32FC1));
  training_labels_ = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::Size(1,(int)all_pix),CV_32SC1));

  //load the separate training data
  LoadImages(positive_set.image_urls(), positive_set.mask_urls(), positive_set.type_);
  LoadImages(negative_set.image_urls(), std::vector<std::string>(), negative_set.type_);
  LoadImages(normal_set.image_urls(), normal_set.mask_urls(), normal_set.type_);
  
}
