#ifndef _RANDOMFOREST_HPP_
#define _RANDOMFOREST_HPP_

#include "baseclassifier.hpp"

namespace ttrk{

  /**
   * @class RandomForest
   * @brief The random forest classification.
   *
   * The class for handling classification with a random forest classifier. Can handle large numbers of classes.
   */

  class RandomForest : public BaseClassifier {

  public: 
    

    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Vec3b &pixel) const;

    /**
     * A probabilistic prediction on the class a pixel belongs to.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs all others.
     */
    virtual float PredictProb(const cv::Vec3b &pixel, const size_t class_index) const;


  };
   



}

#endif
