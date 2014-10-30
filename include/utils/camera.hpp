#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__
#include "../headers.hpp"
#include "../../deps/image/image/image.hpp"

namespace ttrk{

  class StereoCamera;

  /**
  * @class MonocularCamera
  * @brief A class representing a monocular camera. Projects points onto the image plane and generates unprojected rays from pixels.
  * This camera setup mirrors an OpenGL setup where the right handed (in world space) camera points down the -z direction.
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
    * @param[in] distortion The distortion parameters of the camera.
    * @param[in] image_width The width of the image plane in pixels.
    * @param[in] image_width The height of the image plane in pixels.
    */
    MonocularCamera(const cv::Mat &intrinsic, const cv::Mat &distortion, const int image_width, const int image_height);

    /**
    * Construct a camera setting its parameters to /f$f_{x} = 1000, f_{y} = 1000, c_{x} = 0.5\times ImageWidth, c_{y} = 0.5\times ImageHeight \f$.
    */
    MonocularCamera(){}
    
    /**
    * Destructor for the camera.
    */
    virtual ~MonocularCamera(){};

    void SetGLProjectionMatrix() const;
    void SetupCameraForDrawing(const int viewport_width, const int viewport_height) const;

    cv::Mat GetUnprojectedImagePlane(const int width, const int height);

    /** 
    * Project a 3D point onto the image plane without rounding its coordinates to a specific pixel.
    * @param point The 3D point to project.
    * @return The projected point.
    */
    cv::Point2d ProjectPoint(const cv::Point3d &point) const;
    
    /**
    * Project a 3D point directly to an image pixel by nearest neighbour interpolation.
    * @param point The 3D point to project.
    * @return The projected point.
    */
    cv::Point2i ProjectPointToPixel(const cv::Point3d &point) const;

    /**
    * Unproject a pixel to a ray into space.
    * @param[in] point The pixel coordinates.
    * @return The ray.
    */
    cv::Point3d UnProjectPoint(const cv::Point2i &point) const;

    double Fx() const { return fx_; }
    double Fy() const { return fy_; }
    double Px() const { return px_; }
    double Py() const { return py_; }
    int Width() const { return image_width_; }
    int Height() const { return image_height_; }
    inline cv::Mat CameraMatrix() const;
    friend class StereoCamera;

  protected:
      
    float fx_; /**< The camera focal length in units of horizontal pixel length. */
    float fy_; /**< The camera focal length in units of vertical pixel length. */
    float px_; /**< The camera horizontal principal point in units of horizontal pixel length. */
    float py_; /**< The camera horizontal principal point in units of vertical pixel length. */
    cv::Mat distortion_params_; /**< The camera distortion parameters. */

    int image_width_; /**< The width of the image plane in pixels. */
    int image_height_; /**< The height of the image plane in pixels. */

    ci::Vec3f camera_center_; /**< The center of the camera with respect to the camera coordinate system (useful for offsetting a camera in stereo/trinocular rig). */
    ci::Matrix33f rotation_; /**< The orientation of the camera with respect to the camera coordinate system (useful for offsetting a camera in stereo/trinocular rig). */

  private:

    cv::Mat unprojected_image_; /**< The z=1 unprojected image plane. */

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
    boost::shared_ptr<MonocularCamera> left_eye() const { return left_eye_; }
    
    /**
    * Get a reference to the right eye of the camera.
    * @return The right eye of the camera.
    */
    boost::shared_ptr<MonocularCamera> right_eye() const { return right_eye_; }
    
    void SetupGLCameraFromLeft() const;
    void SetupGLCameraFromRight() const;

    inline ci::Matrix33f ciExtrinsicRotation() const { return right_eye_->rotation_; }
    inline ci::Vec3f ciExtrinsicTranslation() const { return right_eye_->camera_center_; }
    inline cv::Mat cvExtrinsicRotation() const;
    inline cv::Vec3d cvExtrinsicTranslation() const;
    
  protected:

    boost::shared_ptr<MonocularCamera> left_eye_; /**< The left camera of the stereo rig. */
    boost::shared_ptr<MonocularCamera> right_eye_; /**< The right camera of the stereo rig. */

  private:

    StereoCamera(){};

  };


}


#endif //CAMERA_HPP_
