#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <cinder/gl/Light.h>

#include "../headers.hpp"
#include "image.hpp"

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

    /**
    * Setup the camera before drawing using it. Changes GL_MODELVIEW, GL_PROJECTION and GL_LIGHTING.
    */
    void SetupCameraForDrawing() const;

    /**
    * Resets the camera state.
    */
    void ShutDownCameraAfterDrawing() const;

    /**
    * Generates the unprojected ray from each pixel for a region of size width and height. Uses caching for speed.
    * @param[in] width Width of the image to work with.
    * @param[in] height Height of the image to work with.
    * @return The uprojected ray image.
    */
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
    * Project a 3D point directly to an image pixel by nearest neighbour interpolation.
    * @param point The 3D point to project.
    * @return The projected point.
    */
    ci::Vec2i ProjectPointToPixel(const ci::Vec3f &point) const;

    /**
    * Unproject a pixel to a ray into space.
    * @param[in] point The pixel coordinates.
    * @return The ray.
    */
    cv::Point3d UnProjectPoint(const cv::Point2i &point) const;

    /**
    * Setup the OpenGL light, positioning at the camera centre.
    */
    void SetupLight();
    
    /**
    * Camera focal length in x pixel dimensions.
    * @return The focal length.
    */
    float Fx() const { return fx_; }

    /**
    * Camera focal length in y pixel dimensions.
    * @return The focal length.
    */
    float Fy() const { return fy_; }
    
    /**
    * Camera principal point in the x dimension.
    * @return The principal point.
    */
    float Px() const { return px_; }

    /**
    * Camera principal point in the y dimension.
    * @return The principal point.
    */
    float Py() const { return py_; }

    /**
    * Get image width.
    * @return The image width.
    */
    int Width() const { return image_width_; }
    
    /**
    * Get image height.
    * @return The image height.
    */
    int Height() const { return image_height_; }

    /**
    * Generate the camera matrix.
    * @return The camera matrix.
    */
    inline cv::Mat CameraMatrix() const;

    friend class StereoCamera;

  protected:
      
    /**
    * Create and set the matrix GL_PROJECTION from the camera calibration matrix.
    */
    void SetGLProjectionMatrix() const;

    float fx_; /**< The camera focal length in units of horizontal pixel length. */
    float fy_; /**< The camera focal length in units of vertical pixel length. */
    float px_; /**< The camera horizontal principal point in units of horizontal pixel length. */
    float py_; /**< The camera horizontal principal point in units of vertical pixel length. */
    cv::Mat distortion_params_; /**< The camera distortion parameters. */

    int image_width_; /**< The width of the image plane in pixels. */
    int image_height_; /**< The height of the image plane in pixels. */

    ci::Vec3f camera_center_; /**< The center of the camera with respect to the camera coordinate system (useful for offsetting a camera in stereo/trinocular rig). */
    ci::Vec3f world_up_; /**< The world up vector. */
    ci::Vec3f look_at_; /**< The camera look at vector. */
   
    ci::Matrix33f rotation_; /**< The orientation of the camera eye relative to world coordinates. */

    boost::shared_ptr<ci::gl::Light> light_; /**< Each camera has a ref to the scene light to shine on things. */

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

    /**
    * Transform a point from the right eye coordinates to the left eye coordinates.
    * @param[in] point_in_right_eye_coords The point in the coordinates of the right eye.
    * @return The point in the left eye coordinates.
    */
    ci::Vec3f  TransformPointFromRightToLeft(const cv::Vec3f &point_in_right_eye_coords) const { return TransformPointFromRightToLeft(ci::Vec3f(point_in_right_eye_coords[0], point_in_right_eye_coords[1], point_in_right_eye_coords[2])); }

    /**
    * Transform a point from the right eye coordinates to the left eye coordinates.
    * @param[in] point_in_right_eye_coords The point in the coordinates of the right eye.
    * @return The point in the left eye coordinates.
    */
    ci::Vec3f TransformPointFromRightToLeft(const ci::Vec3f &point_in_right_eye_coords) const;

    /**
    * Transform a point from the left eye coordinates to the right eye coordinates.
    * @param[in] point_in_left_eye_coords The point in the coordinates of the left eye.
    * @return The point in the right eye coordinates.
    */
    ci::Vec3f  TransformPointFromLeftToRight(const cv::Vec3f &point_in_left_eye_coords) const { return TransformPointFromLeftToRight(ci::Vec3f(point_in_left_eye_coords[0], point_in_left_eye_coords[1], point_in_left_eye_coords[2])); }
    
    /**
    * Transform a point from the left eye coordinates to the right eye coordinates.
    * @param[in] point_in_left_eye_coords The point in the coordinates of the left eye.
    * @return The point in the right eye coordinates.
    */
    ci::Vec3f TransformPointFromLeftToRight(const ci::Vec3f &point_in_left_eye_coords) const;

    /**
    * Accessor for the camera right eye orientation relative to left eye. This does not transform points from the left to the right coordinate system, rather it's the orientation of the right coordinate system w.r.t the left.
    * @return The right eye orientation.
    */
    inline ci::Matrix33f ciExtrinsicRotation() const { return right_eye_->rotation_; }

    /**
    * Accessor for the camera right eye translation relative to left eye. This does not transform points from the left to the right coordinate system, rather it's the orientation of the right coordinate system w.r.t the left.
    * @return The right eye translation.
    */
    inline ci::Vec3f ciExtrinsicTranslation() const { return right_eye_->camera_center_; }

    /**
    * Accessor for the camera right eye orientation relative to left eye. This does not transform points from the left to the right coordinate system, rather it's the orientation of the right coordinate system w.r.t the left.
    * @return The right eye orientation.
    */
    inline cv::Mat cvExtrinsicRotation() const;

    /**
    * Accessor for the camera right eye translation relative to left eye.
    * @return The right eye translation.
    */
    inline cv::Vec3d cvExtrinsicTranslation() const;
    
  protected:

    boost::shared_ptr<MonocularCamera> left_eye_; /**< The left camera of the stereo rig. */
    boost::shared_ptr<MonocularCamera> right_eye_; /**< The right camera of the stereo rig. */

  private:

    StereoCamera(){};

  };


}


#endif //CAMERA_HPP_
