#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/gl/Texture.h>

#include "../localizer.hpp"
#include "../../../utils/camera.hpp"
#include "../../../utils/image.hpp"
#include "../../model/pose.hpp"
#include "../../../utils/plotter.hpp"
#include "../features/register_points.hpp"
#include "../features/lk_tracker.hpp"

namespace ttrk {

  /**
  * @class PWP3D
  * @brief An abstract base class to do most of the PWP3D tracking functionality. Specialized by monocular and stereo versions for excact cost function update.
  */
  class PWP3D : public Localizer, public ErrorMetricPlottable {
    
  public:

    /**
    * Default constructor. Sets up internal framebuffers etc.
    * @param[in] width The width of the window/framebuffer.
    * @param[in] height The height of the window/framebuffer.
    */
    PWP3D(const int width, const int height);

    /**
    * Destructor.
    */
    ~PWP3D();

    /**
    * Load the shaders we use to compute the projections and contours for the pose estimation.
    */
    void LoadShaders();

    /**
    * Construct a signed distance function and the intersection images which hold the 3D points in camera space which each point on the image plane projects to. Points outside the contour project to GL_FAR.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model used for the projection.
    * @param[out] sdf_image A 32 bit single channel floating point image of the distance from each pixel to the contour. Positive values inside contour and negative values outside.
    * @param[out] front_intersection_image A 32 bit 3 channel image of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] front_intersection_image A 32 bit 3 channel image of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    */
    virtual void ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) ;
    
    /**
    * Render the mesh in the current pose getting the depth of the each pixel and the outer contour.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model to use for the projection.
    * @param[out] front_depth A 32 bit single channel image of the depth of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] back_depth A 32 bit single channel image of the depth of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] contour An 8 bit single channel image which is 0 at every point that is not on the outer contour of the projected mesh and 255 where it is.
    */
    virtual void RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour) ;

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] classification_image The classification image
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    * @param[in] fg_area The area of the foreground region.
    * @param[in] bg_area The area of the background region.
    * @return The region agreement value.
    */
    virtual float GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf, const float fg_area, const float bg_area) const;
    
    /** 
    * Update the jacobian by computing the second part of the derivative and multiplying by the region agreement.
    * @param[in] region_agreement Whether the pixel in question agrees with the foreground probability model.
    * @param[in] sdf The value of the signed distance function at the pixel in question.
    * @param[in] dsdf_dx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dsdf_dy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel
    * @param[in] fx The camera focal length in x-pixel units.
    * @param[in] fy The camera focal length in y-pixels units.
    * @param[in] front_intersection_point The 3D point on the near side of the mesh that the current pixel projects to. For pixels that project just miss the contour the closest intersection point is chosen (mathematical hack...).
    * @param[in] back_intersection_point  The 3D point on the far side of the mesh mesh that the current pixel projects to.
    * @param[in] model The currently tracked model. This is used to compute the jacobian.
    * @param[out] jacobian The current jacobian values, these are updated.
    */
    void UpdateJacobian(const float region_agreement, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_image, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, 7> &jacobian);


    /**
    * Update the point registration jacobian, finding the updates to the pose parameters that minimize the point to point error.
    * @param[in] current_model The model to align.
    * @param[in] jacobian The jacobian to update.
    * @param[in] hessian_approx The hessian approximation if using GN.
    */
    void ComputePointRegistrationJacobian(boost::shared_ptr<Model> current_model, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx);
    void ComputeLKJacobian(boost::shared_ptr<Model> current_model, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx);

    /**
    * Find the closest intersection point for pixels which project 'very close' to the target mesh. This is done by searching the sdf_im for the closest zero value to (r,c).
    * @param[in] sdf_im A pointer to the image data for the Signed Distance Function image.
    * @param[in] r The row value of the current pixel.
    * @param[in] c The column value of the current pixel.
    * @param[in] height The height of the image we are using.
    * @param[in] width. The width of the image we are using.
    * @param[out] closest_r The output row of the pixel in the signed distance function which has zero value which is closest to (r,c).
    * @param[out] closest_c The output column of the pixel in the signed distance function which has zero value which is closest to (r,c).
    * @return The success of the search.
    */
    bool FindClosestIntersection(const float *sdf_im, const int r, const int c, const int height, const int width, int &closest_r, int &closest_c) const;

    /**
    * Scale the jacobian for the rigid pose parameters when doing gradient descent.
    * @param[in] jacobian The jacobian to be scaled.
    * @return The scaled jacobian.
    */
    std::vector<float> ScaleRigidJacobian(cv::Matx<float, 7, 1> &jacobian) const;

    /**
    * Compute a smoothed heaviside function output for a given value.
    * @param[in] x The input value.
    * @return The value scaled to between 0-1 with a smoothed logistic function manner.
    */
    float HeavisideFunction(const float x) const {
      return 0.5f*(1.0f + x / float(HEAVYSIDE_WIDTH) + (1.0f / float(M_PI))*sin((float(M_PI)*x) / float(HEAVYSIDE_WIDTH)));
      //return 0.5f + 0.5f*tanh(float(HEAVYSIDE_WIDTH)*x);
    }

    /**
    * Compute a smoothed delta function output for a given value. This is basically a Gaussian approximation where the standard deviation is close to zero.
    * Warning this approximation is only valid close to the zero line.
    * @param[in] x The input value.
    * @return The output value.
    */
    float DeltaFunction(const float x) const {
      return (1.0f / 2.0f / HEAVYSIDE_WIDTH*(1.0f + cos(float(M_PI)*x / HEAVYSIDE_WIDTH)));
      //return (0.5 * HEAVYSIDE_WIDTH) / (cosh(HEAVYSIDE_WIDTH * x)*cosh(HEAVYSIDE_WIDTH*x));
    }
    
    /**
    * Set the reference to the current image. 
    * @param[in] frame The new frame to process.
    */
    void SetFrame(boost::shared_ptr<sv::Frame> frame) { frame_ = frame; }
  
    /**
    * Test for convergence (currently just uses number of steps).
    * @return Whether convergence has been reached.
    */
    virtual bool HasConverged() const { return curr_step == NUM_STEPS; }

    /**
    * Accessor for the heaviside function.
    * @return The heaviside width.
    */
    int GetHeavisideWidth() const { return HEAVYSIDE_WIDTH; }

    /**
    * Get the error value for the current image.
    * @param[in] classification_image The classification image for this frame.
    * @param[in] row_idx The x-pixel coordinate for this image.
    * @param[in] col_idx The y-pixel coordinate for this image.
    * @param[in] sdf_value The signed distance function value for this pixel.
    * @param[in] target_label The target label for multiclass classification.
    * @param[in] fg_area The area of the foreground region.
    * @param[in] bg_area The area of the background region.
    * @return The error value.
    */
    float GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const int target_label, const float fg_area, const float bg_area) const;

  protected:

    /**
    * Compute the areas of the foreground and background regions.
    * @param[in] sdf The signed distance function image.
    * @param[out] fg_area The foreground area.
    * @param[out] bg_area The background area.
    * @param[out] contour_area The area of non-zero (or non-ludicrously-small) values from the signed distance function.
    */
    void ComputeAreas(const cv::Mat &sdf, float &fg_area, float &bg_area, size_t &contour_area);

    boost::shared_ptr<sv::Frame> frame_; /**< Just a reference to the current frame, probably not really useful, may be removed. */
    
    ci::gl::Fbo front_depth_framebuffer_; /**< Framebuffer to write the front depth values into. */
    ci::gl::Fbo back_depth_framebuffer_; /**< Framebuffer to write the back depth values into. Has 2 colour buffers. */

    ci::gl::GlslProg front_depth_;  /**< Shader to compute the front depth buffer. */
    ci::gl::GlslProg back_depth_and_contour_;  /**< Shader to compute the back depth buffer and contour. */

    size_t NUM_STEPS;  /**< Number of step for the optimization. */
    size_t curr_step; /**< Current step in the optimization. */
    
    int HEAVYSIDE_WIDTH;  /**< Width of the heaviside blurring function. */

    boost::shared_ptr<PointRegistration> point_registration_; /**< Computes the point registration error. */
    
    boost::shared_ptr<LKTracker> lk_tracker_;


  };




}


#endif
