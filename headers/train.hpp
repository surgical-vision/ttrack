#ifndef _TRAIN_HPP_
#define _TRAIN_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include <opencv/ml/ml.hpp>

namespace ttrk{

  /**
   * @class Train
   * @brief Abstract base class of Training system. Is owned by the detection system and carries out some form of training for it
   *
   */

  class Train{

    /**
     * Train the classifier.
     */

    void TrainClassifier();

    /**
     * This loops through the image directories getting the filenames and also getting the image sizes to preallocate the training data matrices. 
     */

    void LoadTrainingData();

    /**
     * Load the training examples. This should include masks if 
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

    void AnalyseTrainData();


    void DebugTest(const std::string &infile, const std::string &outfile);
    void DebugTest(const cv::Mat &in, cv::Mat &out);


  protected:
    
    cv::Mat training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    cv::Mat training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */  

    CvRTrees classifier_; /**< The OpenCV Random Forest classifier. */
    
    std::string root_dir_; /**< The root directory of the training data suite. */

    enum LoadType {POSITIVE,NEGATIVE,BOTH};
    


  };

}
