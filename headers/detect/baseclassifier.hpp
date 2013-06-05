#ifndef _BASE_CLASSIFIER_HPP_
#define _BASE_CLASSIFIER_HPP_

#include "../headers.hpp"
#include <opencv2/ml/ml.hpp>
#include <boost/filesystem.hpp>

namespace ttrk{
  
  /**
   * An enum to allow different colourspaces to be switched on and off within the classifier using a bitwise OR.
   */
  enum Colour{
    BLUE = 0x1,
    GREEN = 0x2,
    RED = 0x4,
    HUE = 0x8,
    SAT = 0x10,
    VALUE = 0x20,
    X = 0x40,
    Y = 0x80,
    Z = 0x100,
    NR = 0x200,
    NG = 0x400,
    OPP2 = 0x800,
    OPP3 = 0x1000,
    INVALID_FLAG = 0x2000
  };

  /**
   * @class BaseClassifier
   * @brief An abstract class for classification algorithms.
   *
   * The interface that must be provided by any classification algorithm to perform prediction, save and load the parameters of a trained system.
   */

  class BaseClassifier {

  public:

    /**
     * A function for training the classifier of choice. Will accept data in the form returned by the TrainData class.
     * @param[in] training_data The training data to be used for training.
     * @param[in] labels The class labels for each training sample.
     */
    virtual void TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir) = 0;

    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The pixel from the NDImage class.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Mat &pixel) const = 0;

    /**
     * A probabilistic prediction on the class a pixel belongs to.
     * @param[in] pixel The pixel from the NDImage class.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs all others.
     */
    virtual float PredictProb(const cv::Mat &pixel, const size_t class_index) const = 0;
    
    /**
     * Wrapper function for loading the classifier from an xml file.
     * @param[in] url The address of the file.
     */
    virtual void Load(const std::string &url) = 0;

    /**
     *
     *
     */
     virtual void Save(const std::string &url) const = 0;

    /**
     * Get the name of the classifier type as a string. This can be useful
     * for saving or loading the classifier with a sensible name.
     * @return The classifier name as a string.
     */
    virtual std::string NameAsString() const = 0;


    /**
     * Convenience function for classifying a whole image at once.
     * @param im The image to classify. This image is modified to 
     */
    //void PredictClass(cv::Mat &im) const;
    //void PredictProb(cv::Mat &im, const size_t class_index) const;
    
    BaseClassifier();
    virtual ~BaseClassifier();

  protected:

    Colour colourspace_; /**< Colourspace settings for the classifier. */

    size_t var_mask_; /**< Bitmask for specifying which Colorspaces/features to use. */
    
    

  };

}
#endif