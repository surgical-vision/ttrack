#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

//#include <ceres/ceres.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/gl/Texture.h>

#include "../localizer.hpp"
#include "../../utils/camera.hpp"
#include "../../../deps/image/image/image.hpp"
#include "../pose.hpp"

namespace ttrk {

  class PWP3D : public Localizer {

  public: 
    
  protected:

    /**
    * Default constructor. Sets up internal framebuffers etc.
    * @param[in] width The width of the window/framebuffer.
    * @param[in] height The height of the window/framebuffer.
    */
    PWP3D(const int width, const int height);

    /**
    * Load the shaders we use to compute the projections and contours for the pose estimation.
    */
    virtual void LoadShaders();

    /**
    * Construct a signed distance function and the intersection images which hold the 3D points in camera space which each point on the image plane projects to. Points outside the contour project to GL_FAR.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model used for the projection.
    * @param[out] sdf_image A 32 bit single channel floating point image of the distance from each pixel to the contour. Positive values inside contour and negative values outside.
    * @param[out] front_intersection_image A 32 bit 3 channel image of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] front_intersection_image A 32 bit 3 channel image of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    */
    void ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);
    
    /**
    * Render the mesh in the current pose getting the depth of the each pixel and the outer contour.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model to use for the projection.
    * @param[out] front_depth A 32 bit single channel image of the depth of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] back_depth A 32 bit single channel image of the depth of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] contour An 8 bit single channel image which is 0 at every point that is not on the outer contour of the projected mesh and 255 where it is.
    */
    void RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour);

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    */
    float GetRegionAgreement(const int r, const int c, const float sdf);

    /** 
    * Update the jacobian by computing the second part of the derivative and multiplying by the region agreement.
    * @param[in] region_agreement Whether the pixel in question agrees with the foreground probability model.
    * @param[in] dsdf_dx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dsdf_dy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel
    * @param[in] front_intersection_point The 3D point on the near side of the mesh that the current pixel projects to. For pixels that project just miss the contour the closest intersection point is chosen (mathematical hack...).
    * @param[in] back_intersection_point  The 3D point on the far side of the mesh mesh that the current pixel projects to.
    * @param[out] jacobian The current jacobian values, these are updated.
    * @param[in] num_dofs The number of degrees of freedom in the jacobian.
    */
    void UpdateJacobian(const float region_agreement, const float dsdf_dx, const float dsdf_dy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_image, float *jacobian, const int num_dofs);

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
    * Compute a smoothed heaviside function output for a given value.
    * @param[in] x The input value.
    * @return The value scaled to between 0-1 with a smoothed logistic function manner.
    */
    float HeavisideFunction(const float x){
      return 0.5f*(1.0f + x / float(HEAVYSIDE_WIDTH) + (1.0 / M_PI)*sin((M_PI*x) / float(HEAVYSIDE_WIDTH)));
    }

    /**
    * Compute a smoothed delta function output for a given value. This is basically a Gaussian approximation where the standard deviation is close to zero.
    * @param[in] x The input value.
    * @return The output value.
    */
    float DeltaFunction(const float x){
      return (1.0f / 2.0f / HEAVYSIDE_WIDTH)*(1.0f + cos(M_PI*x / HEAVYSIDE_WIDTH));
    }

    //bool HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const ;

    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    * @param[in] step_id The number of steps done. Used for scaling down the step size.
    */
    //void ScaleJacobian(PoseDerivs &jacobian, const size_t step_number, const size_t pixel_count) const;

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in]    jacobian The pose update of the target object.
    */    
    //void ApplyGradientDescentStep(PoseDerivs &jacobian, Pose &pose, const size_t step,  const size_t pixel_count);



    /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    * @return bool The success of the intersection test.
    */
    //bool GetTargetIntersections(const int r, const int c, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const ;

    //bool GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const;

    boost::shared_ptr<sv::Frame> frame_;
    ci::gl::Fbo front_depth_framebuffer_; /**< Framebuffer to write the front depth values into. */
    ci::gl::Fbo back_depth_framebuffer_; /**< Framebuffer to write the back depth values into. Has 2 colour buffers. */

    ci::gl::GlslProg front_depth_;
    ci::gl::GlslProg back_depth_and_contour_;

    std::string DEBUG_DIR_;

    size_t NUM_STEPS;
    int HEAVYSIDE_WIDTH;

  };




}


#endif
