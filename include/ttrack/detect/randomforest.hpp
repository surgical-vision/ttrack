#ifndef _RANDOMFOREST_HPP_
#define _RANDOMFOREST_HPP_

#include "baseclassifier.hpp"

namespace ttrk{

  /**
   * @class RandomForest
   * @brief The random forest classifier.
   *
   * The class for handling classification with a random forest classifier. Can handle large numbers of classes.
   */

  class RandomForest : public BaseClassifier {

  public: 
    
    /**
     * A function for training the classifier of choice. Will accept data in the form returned by the TrainData class.
     * @param[in] training_data The training data to be used for training.
     * @param[in] labels The class labels for each training sample.
     * @param[in] root_dir The root directory with the location where the classifier should be saved.
     */
    virtual void TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir);

    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The pixel from the NDImage.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Mat &pixel) const;

    /**
     * A probabilistic prediction on the class a pixel belongs to.
     * @param[in] pixel The pixel from the NDImage.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs all others.
     */
    virtual float PredictProb(const cv::Mat &pixel, const size_t class_index) const;

    /**
     * A load function for loading the Random Forest from an xml file.
     * @param[in] url The file url.
     */
    virtual void Load(const std::string &url);

    virtual void Save(const std::string &url) const;

    /**
     * Get the name of the random forest as a string: "random_forest".
     * This is useful for saving or loading the classifier from a directory.
     * @return The string "random_forest"
     */
    virtual std::string NameAsString() const;

    /**
    * Check if the loaded classifier supports classifying multiple classes or if it just does binary classification.
    * @return Whether the classifier supports mulitple classes or not.
    */
    virtual bool IsBinary() const override { return true; }


  protected:

    CvRTrees forest_; /**< The internal representation of a random forest. */

  };
   



}

#endif
