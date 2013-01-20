#ifndef _CROSS_VALIDATE_HPP_
#define _CROSS_VALIDATE_HPP_
#include "train.hpp"

namespace ttrk{

  /**
   * @class CrossValidate
   * @brief A class for handling training a classifier using A cross validation training
   * system. 
   *
   * The methods of the class are called in the constructor so it is only necessary to construct the object. It automatically handles the rest.
   */

  class CrossValidate : public Train {

  public:
    
    /**
     * The constructor uses the root_dir url to find the relevant images by making some basic
     * assumptions about the directory tree. It then loads the pixels into the training
     * matrices and trains the BaseClassifier. Currently it saves the classifier from the fold 
     * which had the lowest training error.
     * @param[in] root_dir The root directory contining the subfolders where the images are found.
     * @param[out] to_train The classifier to train.
     * @param[in] num_folds The number of folds.
     */

    CrossValidate(boost::shared_ptr<std::string> root_dir, BaseClassifier &to_train, const int num_folds); //pick num_folds based on size
  
  protected:
    
    /**
     * Loads the training data.
     */
    virtual void LoadTrainingData();
    
    /**
     * Loads the data for the nth fold from the entire dataset into training and testing sets.
     */   
    void GetFoldMatrices(const int fold, boost::shared_ptr<cv::Mat> train, boost::shared_ptr<cv::Mat> label ,boost::shared_ptr<cv::Mat> test, boost::shared_ptr<cv::Mat> truth);

    int num_folds_; /**<  The number of folds we are using. */

  };



}

#endif
