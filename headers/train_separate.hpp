#ifndef _TRAIN_SEPARATE_HPP_
#define _TRAIN_SEPARATE_HPP_
#include "train.hpp"

namespace ttrk{

  /**
   * @class TrainSeparate
   * @brief A class for handling training a classifier using training data that cannot be tested on. These could 
   * be images of the target object against a plain background for easy segmentation.
   * 
   * The methods of the class are called in the constructor so it is only necessary to construct the object. It automatically handles the rest.
   */

  class TrainSeparate : public Train {

  public:

   /**
    * The constructor uses the root_dir url to find the relevant images by making some basic
    * assumptions about the directory tree. It then loads the pixels into the training
    * matrices and trains the BaseClassifier. 
    * @param[in] root_dir The root directory contining the subfolders where the images are found.
    * @param[out] to_train The classifier to train.
    */
    TrainSeparate(boost::shared_ptr<std::string> root_dir, BaseClassifier &to_train);

  protected:

   /**
    * Loads the training data.
    */
    virtual void LoadTrainingData();
    
  };


}


#endif
