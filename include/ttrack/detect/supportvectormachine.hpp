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

    virtual void TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir);
    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The pixel from the NDImage.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Mat &pixel) const;

    /**
     * A probabilistic prediction on the class a pixel belongs to. Obviously these are just x and (1-x) for an SVM.
     * @param[in] pixel The pixel from the NDImage.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs the other.
     */
    virtual float PredictProb(const cv::Mat &pixel, const size_t class_index) const;

    /**
     * A load function for loading the SVM from an xml file.
     * @param[in] url The file url.
     */
    virtual void Load(const std::string &url);
    virtual void Save(const std::string &url) const;
    /**
     * Return the string "support_vector_machine" which can be useful for saving or loading
     * the classifier from a directory.
     * @return The name of the classifier type as a string.
     */
    virtual std::string NameAsString() const;

    /**
    * Check if the loaded classifier supports classifying multiple classes or if it just does binary classification.
    * @return Whether the classifier supports mulitple classes or not.
    */
    virtual bool IsBinary() const override { return true; }

  };


}

#endif
