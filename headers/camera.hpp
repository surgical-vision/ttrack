#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_
#include "headers.hpp"

namespace ttrk{

  class MonocularCamera {

  public:
   
    explicit MonocularCamera(const std::string &calibration_filename);
    MonocularCamera(const cv::Mat &intrinsic, const cv::Mat &distortion):intrinsic_matrix_(intrinsic),distortion_params_(distortion){}
    MonocularCamera(){}
    virtual ~MonocularCamera(){};
    
    cv::Point2f ProjectPoint(const cv::Point3f &point) const;
    cv::Point2i ProjectPointToPixel(const cv::Point3f &point) const;
    //project to image plane
    //unproject to z = 1
    //unproject to ray

    
  protected:
      
    cv::Mat intrinsic_matrix_; 
    cv::Mat distortion_params_;

  private:


  };



  class StereoCamera {

  public:

    explicit StereoCamera(const std::string &calibration_filename);

    const MonocularCamera &left_eye() const { return left_eye_; }
    const MonocularCamera &right_eye() const { return right_eye_; }
    
    //get epipolar line

    //reproject to 3d point


  protected:

    MonocularCamera left_eye_;
    MonocularCamera right_eye_;

    cv::Mat extrinsic_matrix_;

  private:

    StereoCamera(){};

  };


}


#endif //CAMERA_HPP_
