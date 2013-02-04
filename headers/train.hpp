#ifndef _TRAIN_HPP_
#define _TRAIN_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include "helpers.hpp"
#include "baseclassifier.hpp"
#include "im_mask_set.hpp"
#include <boost/shared_ptr.hpp>
#include <opencv2/ml/ml.hpp>
#include <unordered_map>

namespace ttrk{

  /**
   * An enum to specify which load type is being used in the training system.
   */
  //enum LoadType {POSITIVE,NEGATIVE,BOTH};

  /**
   * @class TrainData
   * @brief Abstract base class of Training system. Is owned by the detection system and loads training data 
   * into a form which can be used by the classifier for training. Derived 
   */

  class Train{

  public:

    Train(boost::shared_ptr<std::string> root_dir, const std::string &class_file = "./config/classes.xml");
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
     * @param[in] type Whether the training examples are positive or negative or normal images.
     */
    void LoadImages(const std::vector<std::string> &image_urls, 
                    const std::vector<std::string> &mask_urls,
                    const ImageMaskSet::LoadType type);
   
    /**
     * Loads the pixels from the image and mask according to the training data type.
     * @param[in] nd_image The N dimensional image
     * @param[in] mask The mask specifying which class the pixels belong to.
     * @param[in] type The load type, positive, negative or both.
     * @param[in] index The current index of hte training matrix. Should be initialised to zero before the first call to AddPixels(). As each image is parsed the pixels are added to the current 'end' of the matrix.
     */
    void LoadPixels(const NDImage &nd_image, const cv::Mat &mask, const ImageMaskSet::LoadType type, size_t &index);
    
    boost::shared_ptr<std::string> root_dir_; /**< The root directory of the training data suite. */
    boost::shared_ptr<cv::Mat> training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    boost::shared_ptr<cv::Mat> training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */        

    double recall_; /**< The recall error value of the classifier. */
    double precision_; /**< The precision error value of the classifier. */
    double prob_err_; /**< The probability of error of the classifier. */


    std::unordered_map<cv::Vec3b,size_t,hashVec> class_index_to_mask_; /**< Provies a mapping from the RGB pixel colour in the mask to the integer used to represent the class in the training system. */


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
