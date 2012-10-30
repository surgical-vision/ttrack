#ifndef _TRAIN_HPP_
#define _TRAIN_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include <opencv2/ml/ml.hpp>

namespace ttrk{

  /**
   * An enum to specify which load type is being used in the training system.
   */
  enum LoadType {POSITIVE,NEGATIVE,BOTH};

  /**
   * @class TrainData
   * @brief Class of Training system. Is owned by the detection system and loads training data into
   * a form which can be used by a classifier. It can also perform some basic validation on the 
   * classified data.
   */

  class TrainData{

  public:

    TrainData(std::string &root_dir);
    ~TrainData();

    void LoadCrossValidationData();

    /**
     * This loops through the image directories getting the filenames and also getting the 
     * image sizes to preallocate the training data matrices. This calls LoadImages
     * after allocating the memory for each training matrix.
     */
    void LoadTrainingData(bool cross_validate);

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

    cv::Mat *training_data();
    cv::Mat *training_labels();

  protected:
    
    std::string *root_dir_; /**< The root directory of the training data suite. */
    cv::Mat *training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    cv::Mat *training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */        
    
  private:
     
    TrainData();


  };

  inline cv::Mat *TrainData::training_data(){
    return training_data_;
  }

  inline cv::Mat *TrainData::training_labels(){
    return training_labels_;
  }

}

#endif
