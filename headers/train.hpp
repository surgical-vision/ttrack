#ifndef _TRAIN_HPP_
#define _TRAIN_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include <boost/shared_ptr.hpp>
#include <opencv2/ml/ml.hpp>

namespace ttrk{

  /**
   * An enum to specify which load type is being used in the training system.
   */
  enum LoadType {POSITIVE,NEGATIVE,BOTH};

  /**
   * @class TrainData
   * @brief Class of Training system. Is owned by the detection system and loads training data 
   * into a form which can be used by the classifier for training.
   */

  class TrainData{

  public:

    TrainData(std::string &root_dir);
    ~TrainData();

    /**
     * Loops through the training data directories and loads the image names, preallocates the 
     * matrices and loads the pixels as part of a cross validation training system.
     * Expects a directory data/images and data/masks containing images and masks respectively.
     * Each image-mask pair should have the same filename.
     */
    void LoadCrossValidationData();

    /**
     * This loops through the image directories getting the filenames and also getting the 
     * image sizes to preallocate the training data matrices. This calls LoadImages
     * after allocating the memory for each training matrix.
     * Expects a directory data/positive_data/training_images, data/negative_data/ and data/positive_data/masks containing images and masks respectively.
     * Each image-mask pair should have the same filename. Obviously this doesn't apply for 
     * negative images which do not have an assoicated mask.
     */
    void LoadSeparateTrainingData();

    /**
     * Getter for the training data.
     * @return A shared pointer to the allocated training matrix.
     */
    boost::shared_ptr<cv::Mat> training_data();

    /**
     * Getter for the training labels.
     * @return A shared pointer to the allocated training labels.
     */
    boost::shared_ptr<cv::Mat> training_labels();

    /**
     * Get the nth training set from an N fold cross validation.
     * @param[in] n The fold. Counts from 0.
     * @return A pointer to a training matrix containing the training data.
     */
    void GetFoldMatrices(boost::shared_ptr<cv::Mat> train, boost::shared_ptr<cv::Mat> label,
                         boost::shared_ptr<cv::Mat> test, boost::shared_ptr<cv::Mat> truth, const int fold, const int num_folds);

  protected:

    /**
     * Load the training examples. Iterates through the image urls in the vector and
     * loads the images and masks (depending on LoadType). Constructs a new NDImage from
     * each new image and loads its pixels into the training matrix, setting the response/label
     * accordingly.
     * @param[in] image_urls A vector of the images that are to be loaded.
     * @param[in] mask_urls A vector of masks to be loaded.
     * @param[in] type Wether the training examples are positive or negative.
     */
    void LoadImages(const std::vector<std::string> &image_urls, 
                    const std::vector<std::string> &mask_urls,
                    const LoadType type);
   
    /**
     * Loads the pixels from the image and mask according to the training data type.
     * @param[in] nd_image The N dimensional image
     * @param[in] mask The mask specifying which class the pixels belong to.
     * @param[in] type The load type, positive, negative or both.
     */
    void LoadPixels(const NDImage *nd_image, const cv::Mat &mask, const LoadType type);
    

    std::string *root_dir_; /**< The root directory of the training data suite. */
    boost::shared_ptr<cv::Mat> training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    boost::shared_ptr<cv::Mat> training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */        
    
  private:
     
    TrainData();


  };

  inline boost::shared_ptr<cv::Mat> TrainData::training_data(){
    return training_data_;
  }

  inline boost::shared_ptr<cv::Mat> TrainData::training_labels(){
    return training_labels_;
  }

}

#endif
