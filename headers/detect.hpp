#ifndef _DETECT_HPP_
#define _DETECT_HPP_

#include "headers.hpp"
#include "nd_image.hpp"
#include <opencv2/ml/ml.hpp>

namespace ttrk{

/**
 * @class Detect
 * @brief Detection system. Wraps the classification of images and interface with the TTrack
 * class.
 *
 * This is the class...
 */


  class Detect{
    
  public:

    Detect();
    ~Detect();

    /**
     * The operator overload for the boost thread call. This function wraps the calls to the detection methods.
     * @param[in] image The image pulled from the video file or image file.
     */

    void operator()(cv::Mat *image); 

    /**
     * The operator returns a unique pointer to the classified frame. 
     * @return a unique pointer to the classified frame.
     */
    cv::Mat *GetPtrToClassifiedFrame() const;

    /**
     * Has the detector found a candidate for the object in the frame.
     * @return true if yes, false if no
     */
    bool Found() const;  

    /**
     * Setup the classifier 
     */
    void Setup(const std::string &root_dir);


    /**
     * Train the classifier. Requires a directory of positive images and a directory of negative images
     */
    void Train();

  protected:

    /**
     * This loops through the image directories getting the filenames and also getting the image sizes to preallocate the training data matrices. 
     */

    void LoadTrainingData();

    /**
     * Load the postive training examples. This should include masks.
     */
    void LoadPositive(const std::vector<std::string> &image_urls, 
                      const std::vector<std::string> &mask_urls);
    /**
     * Load the negative training examples. No requirement for masks.
     */
    void LoadNegative(const std::vector<std::string> &image_urls);
   
    /**
     * Loads the pixels from the image and mask.
     */
    void LoadPixels(const NDImage *nd_image, const cv::Mat &mask, const bool positive);

    void AnalyseTrainData();


    std::string root_dir_; /**< A string containing the root directory where classifier, data etc is stored. */
    cv::Mat *frame_; /**< A pointer to the current frame, this is passed to the detector then passed to the tracker. */
    bool found_; /** Indicated whether the target object has been found in the image. */
    
    CvRTrees classifier_; /**< The OpenCV Random Forest classifier. */
    cv::Mat training_data_; /**< Matrix to store the training data. Always of type CV_32FC1. */
    cv::Mat training_labels_; /**< Vector to store the labels for the training data. Always of type CV_32SC1. */  
    NDImage *nd_image_; /**< Data structure to store the multidimensional image with the new features space generated from RGB. */ 

  };

}

#endif

