#ifndef _BASE_CLASSIFIER_HPP_
#define _BASE_CLASSIFIER_HPP_

#include <opencv2/ml/ml.hpp>
#include <boost/filesystem.hpp>

#include "../headers.hpp"
#include "../utils/nd_image.hpp"
#include "../utils/image.hpp"

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
    * Default constructor.
    */
    BaseClassifier();

    /**
    * Default destructor.
    */
    virtual ~BaseClassifier();
    
    /**
    * Classify a frame. This method assumes binary classification problem where the target label is 1. It will return the frame with each pixel
    * labelled with the probability that it takes label 1. As we typically pass in frames with 5 channels (allowing up to 5 labels) this will return
    * an image where the first channel 
    */

    virtual bool ClassifyFrame(boost::shared_ptr<sv::Frame> frame);

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
     * Save the classifier.
     * @param[in] url The url to save the file at.
     */
     virtual void Save(const std::string &url) const = 0;

    /**
     * Get the name of the classifier type as a string. This can be useful
     * for saving or loading the classifier with a sensible name.
     * @return The classifier name as a string.
     */
    virtual std::string NameAsString() const = 0;

    /**
    * Check if the loaded classifier supports classifying multiple classes or if it just does binary classification.
    * @return Whether the classifier supports mulitple classes or not.
    */
    virtual bool IsBinary() const = 0;

  protected:

    Colour colourspace_; /**< Colourspace settings for the classifier. */

    size_t var_mask_; /**< Bitmask for specifying which Colorspaces/features to use. */
    
    

  };

}
#endif
