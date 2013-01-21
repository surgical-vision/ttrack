#ifndef _DETECT_HPP_
#define _DETECT_HPP_

#include "headers.hpp"
#include "baseclassifier.hpp"
#include "nd_image.hpp"
#include "train.hpp"
#include <opencv2/ml/ml.hpp>

namespace ttrk{

  enum ClassifierType {RF,SVM,NBAYES};
  
  enum TrainType {X_VALIDATE,SEPARATE,NA};

/**
 * @class Detect
 * @brief Detection system. Wraps the classification of images and interface with the TTrack
 * class.
 *
 * This is the class that is used to initiate training of a classifier and classify new images.
 */

  
  class Detect{
    
  public:

    /**
     * Construct a detection system and train it.
     * @param[in] root_dir The detection system's root directory. Here it will save/load data. This is shared with the owner ttrack class which can modify the root directory if required. 
     * @param[in] classifier_type The type of classifier to load/create.
     * @param[in] train_type The type of training system to use. For example: cross validation or training/testing on distinct data. Use NA or leave blank if you don't wish to train a classifier.
     */
    Detect(boost::shared_ptr<std::string> root_dir, ClassifierType classifier_type, TrainType train_type=NA);

       
    ~Detect();

    /**
     * The operator overload for the boost thread call. This function wraps the calls to the detection methods.
     * @param[in] image The image pulled from the camera, video file or image file.
     */
    void operator()(boost::shared_ptr<cv::Mat> image); 

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
     * The operator returns a unique pointer to the classified frame. 
     * @return a unique pointer to the classified frame.
     */
    boost::shared_ptr<cv::Mat> GetPtrToClassifiedFrame() const;

  protected:

    void ClassifyFrame();

    void SetHandleToFrame(boost::shared_ptr<cv::Mat> image);

    void ResetHandleToFrame();
    
    /**
     * Dynamically compute the directory being used for saving/loading the classifier. Uses the
     * shared_ptr root directory and appends /classifier/.
     * @return The directory url.
     */
    std::string classifier_dir();

    /**
     * Construct the classifier of choice.
     * @param[in] type The desired type of classification algorithm.
     */
    void SetupClassifier(const ClassifierType type);

    /**
     * Load the classifier from the classifier directory.
     * Call this function after creating an empty classifier with SetupClassifier(type).
     */
    void LoadClassifier();

    boost::shared_ptr<std::string> root_dir_; /**< A string containing the root directory where classifier, data etc is stored. */
    boost::shared_ptr<cv::Mat> frame_; /**< A pointer to the current frame, this is passed to the detector then passed to the tracker. */
    bool found_; /**< Indicated whether the target object has been found in the image. */
    boost::shared_ptr<NDImage> nd_image_; /**< The N-D image which is being tracked. */
    boost::shared_ptr<BaseClassifier> classifier_; /**< The classifier. */
    

  private:
    
    Detect();
   

  };

  inline boost::shared_ptr<cv::Mat> Detect::GetPtrToClassifiedFrame() const {
    return frame_;
  }

  inline bool Detect::Found() const {
    return found_;
  }

  inline bool Detect::Loaded() const {
    return classifier_.use_count() > 0;
  }

  inline std::string Detect::classifier_dir(){
    return *root_dir_ + "/classifier/";
  }

}

#endif

