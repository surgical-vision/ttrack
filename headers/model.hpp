#ifndef _MODEL_HPP_
#define _MODEL_HPP_
#include "headers.hpp"

namespace ttrk{

  template<typename primitive>
  class Model {

  public:
   
    Model(){}
    virtual ~Model(){}
    
    virtual std::vector<primitive> Points() const = 0;
    cv::Mat &Pose() { return pose_ };

  protected:

    cv::Mat pose_;

  };

  template<typename primitive>
  class MISTool : public Model<primitive> {

  public:
    
    MISTool(int width, int height);
    
    virtual std::vector<primitive> Points() const;


  private:

    int width_;
    int height_;




  };

  template<typename primitive>
  MISTool<primitive>::MISTool(int width, int height):width_(width),height_(height){


  }

  template<typename primitive>
  std::vector<primitive> MISTool<primitive>::Points() const {

    std::vector<primitive> points;
    return points;

  }

};


#endif //_MODEL_HPP_
