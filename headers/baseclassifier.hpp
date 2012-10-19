#ifndef _BASE_CLASSIFIER_HPP_
#define _BASE_CLASSIFIER_HPP_

namespace ttrk{

  /**
   * @struct ColourSpace
   * @brief A struct for handling the settings for a classifier which uses a colourspace representation of features.
   *
   * The classifier class contains a series of structs each of which represents a different feature type: colour, texture etc. The struct can be used to switch on and off each colourspace without cluttering the classifiying class.
   *
   */
  
  struct ColourSpace{

    void ToggleAll(const bool toggle){
      if(toggle)
        var_mask_ = BLUE | GREEN | RED | HUE | SAT | VALUE | X | Y | Z | NR | NG | OPP2 | OPP3;
      else 
        var_mask_ = 0;
    }

    enum Color{
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

  };

  class BaseClassifier {

  public:

    /**
     * A discrete prediction on the class a pixel belongs to.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @return The class label.
     */
    virtual size_t PredictClass(const cv::Vec3b &pixel) const = 0;

    /**
     * A probabilistic prediction on the class a pixel belongs to.
     * @param[in] pixel The BGR pixel direct from the image source.
     * @param[in] class_index The index of the class to find the probability of.
     * @return The probability of that class vs all others.
     */
    virtual float PredictProb(const cv::Vec3b &pixel, const size_t class_index) const = 0;
    
    virtual ~BaseClassifier();

  protected:


    ColourSpace cspace_; /**< Colourspace settings for the classifier. */

    uint64_t var_mask_; /**< Bitmask for specifying which Colorspaces/features to use. */
  

  };

}
#endif
