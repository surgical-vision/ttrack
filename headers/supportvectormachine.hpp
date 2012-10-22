#ifndef _SVM_HPP_
#define _SVM_HPP_

#include "baseclassifier.hpp"

namespace ttrk {
  
  /**
   * @class SupportVectorMachine
   * @brief The support vector machine classifier
   *
   * The class for handling classification with a SVM. Can only handle binary classification problems.
   */

  class SupportVectorMachine : public BaseClassifier {

  public:
    
    /**
     * A function for training the classifier of choice. Will accept data in the form returned by the TrainData class.
     * @param[in] training_data The training data to be used for training.
     * @param[in] labels The class labels for each training sample.
     */

    virtual void TrainClassifier(const cv::Mat &training_data,const cv::Mat &labels, const std::string &root_dir);
    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Vec3b &pixel) const;

    /**
     * A probabilistic prediction on the class a pixel belongs to. Obviously these are just x and (1-x) for an SVM.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs the other.
     */
    virtual float PredictProb(const cv::Vec3b &pixel, const size_t class_index) const;

    /**
     * A load function for loading the SVM from an xml file.
     * @param[in] url The file url.
     */
    virtual void Load(const std::string &url);
    


  };


}

#endif
