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
     * Is the detector's classifier loaded?
     * @return True if loaded, false if not.
     */
    bool Loaded() const;

    /**
     * Setup the classifier 
     */
    void Setup(const std::string &root_dir,const bool load_classifier);


    /**
     * Train the classifier. Requires a directory of positive images and a directory of negative images
     */
    void Train();


  protected:


    std::string root_dir_; /**< A string containing the root directory where classifier, data etc is stored. */
    std::string classifier_dir_; /**< A string containing the root directory where the classifier is stored. */
    cv::Mat *frame_; /**< A pointer to the current frame, this is passed to the detector then passed to the tracker. */
    bool found_; /** Indicated whether the target object has been found in the image. */
    
    CvRTrees classifier_; /**< The OpenCV Random Forest classifier. */

    NDImage *nd_image_; /**< Data structure to store the multidimensional image with the new features space generated from RGB. */ 

  };

  inline cv::Mat *Detect::GetPtrToClassifiedFrame() const {
    return frame_;
  }

  inline bool Detect::Found() const {
    return found_;
  }

  inline bool Detect::Loaded() const {
    return true;
  }




}

#endif

