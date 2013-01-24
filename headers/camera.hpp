#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_
#include "headers.hpp"

namespace ttrk{

  class MonocularCamera {

  public:
    
    explicit MonocularCamera(const std::string &calibration_filename);
    virtual ~MonocularCamera(){};
    //project to image plane
    //unproject to z = 1
    //unproject to ray

    
  protected:
      
    cv::Mat intrinsic_matrix_; 
    

  private:

    MonocularCamera(){}



  };



  class StereoCamera {

  public:

    const MonocularCamera &left_eye() const { return left_eye_; }
    const MonocularCamera &right_eye() const { return right_eye_; }
    //get epipolar line

    //reproject to 3d point


  protected:

    MonocularCamera left_eye_;
    MonocularCamera right_eye_;

    cv::Mat extrinsic_matrix_;



  };


}


#endif //CAMERA_HPP_
