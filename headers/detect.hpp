#ifndef _DETECT_HPP_
#define _DETECT_HPP_

#include "headers.hpp"
#include "baseclassifier.hpp"
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
 * This is the class that is used to initiate training of a classifier
 */

  
  class Detect{
    
  public:

    /**
     * Construct a detection system and train it.
     * @param[in] root_dir The detection system's root directory. Here it will save/load data.
     * @param[in] train_type The type of training system to use. For example: cross validation or training/testing on distinct data.
     * @param[in] classifier_type The type of classifier to load/create.
     */
    Detect(const std::string &root_dir, TrainType train_type, ClassifierType classifier_type);

    /**
     * Construct a detection system without specifying a training type. This could be because the classifier is already trained and you just want to load it. 
     * @param[in] root_dir The detection system's root directory. Here it will save/load data.
     * @param[in] classifier_type The type of classifier to load/create.
     */
    Detect(const std::string &root_dir, ClassifierType classifier_type);
    
    ~Detect();

    /**
     * The operator overload for the boost thread call. This function wraps the calls to the detection methods.
     * @param[in] image The image pulled from the camera, video file or image file.
     */

    void operator()(cv::Mat *image); 

    /**
     * Train the classifier using cross validation. Requires a directory of positive images and a directory of negative images.
     * @param[in] nfolds The number of folds to use.
     */
    void TrainCrossValidate(const int nfolds);

    /**
     * Train the classifier using separate data. Requires a directory of positive images and a directory of negative images.
     */
    void TrainSeparate();

    /**
     * Construct the classifier of choice.
     * @param[in] type The desired type of classification algorithm.
     */
    void SetupClassifier(const ClassifierType type);

    /**
     * Load the classifier
     * @param[in] type The type of classifier the user wishes to load. 
     */
    void LoadClassifier(const ClassifierType type);

    /**
     * Setup the classifier root directory. This is where it will find a classifier to load/save and directories to save output images.
     */
    void Setup(const std::string &root_dir);    

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
    cv::Mat *GetPtrToClassifiedFrame() const;

  protected:


    std::string root_dir_; /**< A string containing the root directory where classifier, data etc is stored. */
    std::string classifier_dir_; /**< A string containing the root directory where the classifier is stored. */
    cv::Mat *frame_; /**< A pointer to the current frame, this is passed to the detector then passed to the tracker. */
    bool found_; /** Indicated whether the target object has been found in the image. */
    
    BaseClassifier *classifier_; /**< The classifier. */
    Train *train_; /**< The class for training the classifier. */

  private:
    Detect();
   

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

