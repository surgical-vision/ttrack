#ifndef _BASE_CLASSIFIER_HPP_
#define _BASE_CLASSIFIER_HPP_


struct ColourSpace{

  void ToggleAll(const bool toggle){
    BLUE = RED = GREEN = HUE = SAT = VALUE = X = Y = Z = NR = NG = OPP2 = OPP3 = toggle;
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
    Z = 256,
    NR = 512,
    NG = 1024,
    OPP2 = 2028,
    OPP3 = 4056,
    INVALID_FLAG = 8112
  };

};

class BaseClassifier {

public:
  
  size_t PredictClass(const cv::Vec3b &pixel) const;
  float PredictProb(const cv::Vec3b &pixel) const;

protected:

  uint64_t var_mask_;
  
  





};


#endif
