#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_
#include "../headers.hpp"
#include <image/image.hpp>

namespace ttrk{

  class StereoCamera;

  /**
  * @class MonocularCamera
  * @brief A class representing a monocular camera. Projects points onto the image plane and generates unprojected rays from pixels.
  */
  class MonocularCamera {

  public:
   
    /**
    * Construct a camera with a calibration file. This file should be in the opencv calibration xml format.
    * @param[in] calibration_filename The url of the calibration file.
    */ 
    explicit MonocularCamera(const std::string &calibration_filename);

    /**
    * Construct a camera directly specifying the intrinsic and distortion parameters. 
    * @param[in] intrinsic The intrinsic matrix of the camera.
    * @param[in] distortion The distortion parameters of the camera.
    */
    MonocularCamera(const cv::Mat &intrinsic, const cv::Mat &distortion):intrinsic_matrix_(intrinsic),distortion_params_(distortion){}
    
    /**
    * Construct a camera setting its parameters to /f$f_{x} = 1000, f_{y} = 1000, c_{x} = 0.5\times ImageWidth, c_{y} = 0.5\times ImageHeight \f$.
    */
    MonocularCamera(){}
    
    /**
    * Destructor for the camera.
    */
    virtual ~MonocularCamera(){};
    
    /** 
    * Project a 3D point onto the image plane without rounding its coordinates to a specific pixel.
    * @param point The 3D point to project.
    * @return The projected point.
    */
    cv::Point2f ProjectPoint(const cv::Point3f &point) const;
    
    /**
    * Project a 3D point directly to an image pixel by nearest neighbour interpolation.
    * @param point The 3D point to project.
    * @return The projected point.
    */
    cv::Point2i ProjectPointToPixel(const cv::Point3f &point) const;

    /**
    * Unproject a pixel to a ray into space.
    * @param[in] point The pixel coordinates.
    * @return The ray.
    */
    cv::Point3f UnProjectPoint(const cv::Point2i &point) const;
    
    friend class StereoCamera;

  protected:
      
    cv::Mat intrinsic_matrix_; /**< The internal camera parameters. */
    cv::Mat distortion_params_; /**< The camera distortion parameters. */

  private:


  };


  /**
  * @class StereoCamera
  * @brief An implementation of a stereo camera system. 
  */
  class StereoCamera {

  public:

    /**
    * Construct a stereo camera with a calibration filename. This file should be in opencv xml format specifying the internal and external camera parameters. 
    * @param[in] calibration_filename The url of the calibration file.
    */
    explicit StereoCamera(const std::string &calibration_filename);

    /**
    * Get a reference to the left eye of the camera.
    * @return The left eye of the camera.
    */
    const MonocularCamera &left_eye() const { return left_eye_; }
    const MonocularCamera &rectified_left_eye() const { return rectified_left_eye_; }
    /**
    * Get a reference to the right eye of the camera.
    * @return The right eye of the camera.
    */
    const MonocularCamera &right_eye() const { return right_eye_; }
    const MonocularCamera &rectified_right_eye() const { return rectified_right_eye_; }

    void Rectify(const cv::Size image_size);
    bool IsRectified() const { return rectified_; }
    void RemapLeftFrame(cv::Mat &image) const ;
    void RemapRightFrame(cv::Mat &image) const ;

    void ReprojectTo3D(const cv::Mat &image, cv::Mat &point_cloud,const std::vector<cv::Vec2i> &connected_region) const ;

    cv::Mat GetP1() { return P1(cv::Range(0,3),cv::Range(0,3));}

    void InitRectified() {
      rectified_left_eye_.intrinsic_matrix_ = P1(cv::Range(0,3),cv::Range(0,3));
      rectified_right_eye_.intrinsic_matrix_ = P2(cv::Range(0,3),cv::Range(0,3));
      rectified_left_eye_.distortion_params_ = cv::Mat::zeros(1,5,CV_64FC1);
      rectified_right_eye_.distortion_params_ = cv::Mat::zeros(1,5,CV_64FC1);
    }
  protected:

    bool rectified_;

    MonocularCamera left_eye_; /**< The left camera of the stereo rig. */
    MonocularCamera right_eye_; /**< The right camera of the stereo rig. */
    MonocularCamera rectified_left_eye_;
    MonocularCamera rectified_right_eye_;
    cv::Mat extrinsic_matrix_; /**< The rotation and translation between the image planes of the two cameras. */

    cv::Mat reprojection_matrix_;
    cv::Mat P1,P2,R1,R2;
    cv::Rect roi1, roi2;
    cv::Mat mapx_left_;
    cv::Mat mapy_left_;
    cv::Mat mapx_right_;
    cv::Mat mapy_right_;


  private:

    StereoCamera(){};

  };


}


#endif //CAMERA_HPP_
