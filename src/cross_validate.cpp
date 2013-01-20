#include "../headers/cross_validate.hpp"
#include "../headers/helpers.hpp"
#include "../headers/im_mask_set.hpp"

using namespace ttrk;

CrossValidate::CrossValidate(boost::shared_ptr<std::string> root_dir, BaseClassifier &to_train, const int num_folds):num_folds_(num_folds),Train(root_dir){
  
  LoadTrainingData();
  
  double best_err = 2.0; //cannot be larger than 1.

  for(int i=0;i<num_folds;i++){
    boost::shared_ptr<cv::Mat> train,label,test,truth;
    GetFoldMatrices(i,train,label,test,truth);
    to_train.TrainClassifier(train,label,root_dir);
    TestClassifier(to_train,test,truth);
    const double precision = GetPrecision();
    const double recall = GetRecall();
    const double prob_err = GetPE();
    if(prob_err < best_err){
      best_err = prob_err;
      try{
        to_train.Save(*root_dir + "/classifier/" + to_train.NameAsString() + ".xml" );
      }catch(std::runtime_error &e){
        std::cerr << "Error, could not save classifier.\n" << e.what();
        std::cerr << "Exiting...\n";
        SAFE_EXIT();
      }
    }
  } 
}

void CrossValidate::GetFoldMatrices(const int fold, boost::shared_ptr<cv::Mat> train, boost::shared_ptr<cv::Mat> label ,boost::shared_ptr<cv::Mat> test, boost::shared_ptr<cv::Mat> truth){
  
  const int fold_size = training_data_->rows/num_folds_;
  
  for(int r=0;r<training_data_->rows;r++){

    if(r <= fold*fold_size || r > (fold+1)*fold_size){
     //push row from train and label to nth fold test matrix
      test->push_back(training_data_->rowRange(r,r+1));
      truth->push_back(training_labels_->rowRange(r,r+1));

    }else{
      //push row from train and label to nth fold training matrix
      train->push_back(training_data_->rowRange(r,r+1));
      label->push_back(training_labels_->rowRange(r,r+1));
    
    }
  }

}

void CrossValidate::LoadTrainingData(){

  //find the image names and get the number of pixels required to init the matrices
  ImageMaskSet normal_set(ImageMaskSet::BOTH,*(root_dir_.get()));

  if(normal_set.NumPix() <= 0){
    std::cerr << "No training data found!\n";
    SAFE_EXIT();
  }

  //init the matrices
  training_data_ = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::Size(NDImage::channels_,(int)normal_set.NumPix()),CV_32FC1));
  training_labels_ = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::Size(1,(int)normal_set.NumPix()),CV_32SC1));

  //load all the pixels to the training matrices and set the ground truth values from the masks
  LoadImages(normal_set.image_urls(), normal_set.mask_urls(), normal_set.type_);
  
}
