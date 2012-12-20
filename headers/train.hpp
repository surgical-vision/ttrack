#ifndef _TRAIN_HPP_
#define _TRAIN_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include "baseclassifier.hpp"
#include <boost/shared_ptr.hpp>
#include <opencv2/ml/ml.hpp>

namespace ttrk{

  /**
   * An enum to specify which load type is being used in the training system.
   */
  enum LoadType {POSITIVE,NEGATIVE,BOTH};

  /**
   * @class TrainData
   * @brief Abstract base class of Training system. Is owned by the detection system and loads training data 
   * into a form which can be used by the classifier for training. Derived 
   */

  class Train{

  public:

    Train(boost::shared_ptr<std::string> root_dir);
    ~Train();

    double GetPrecision() const;
    double GetRecall() const;
    double GetPE() const;

  protected:

    void TestClassifier(BaseClassifier &to_train, boost::shared_ptr<cv::Mat>test, boost::shared_ptr<cv::Mat> truth);

    /**
     * Loops through the training data directories and loads the image names, preallocates the 
     * matrices and loads the pixels. 
     */
    virtual void LoadTrainingData() = 0;

    /**
     * Load the training examples. Iterates through the image urls in the vector and
     * loads the images and masks (depending on LoadType). Constructs a new NDImage from
     * each new image and loads its pixels into the training matrix, setting the response/label
     * accordingly.
     * @param[in] image_urls A vector of the images that are to be loaded.
     * @param[in] mask_urls A vector of masks to be loaded.
     * @param[in] type Whether the training examples are positive or negative.
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
    
    boost::shared_ptr<std::string> root_dir_; /**< The root directory of the training data suite. */
    boost::shared_ptr<cv::Mat> training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    boost::shared_ptr<cv::Mat> training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */        

    double recall_; /**< The recall error value of the classifier. */
    double precision_; /**< The precision error value of the classifier. */
    double prob_err_; /**< The probability of error of the classifier. */

  private:
     
    Train();

  };

  inline double Train::GetRecall() const {
    return recall_;
  }

  inline double Train::GetPrecision() const {
    return precision_;
  }

  inline double Train::GetPE() const {
    return prob_err_;
  }
}

#endif
