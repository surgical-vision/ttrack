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
    * Get the second part of the derivative. The derivative of the contour w.r.t the pose parameters.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The value of the signed distance function for (r,c).
    * @param[in] dSDFdx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dSDFdy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel.
    * @return The pose derivitives as a vector.
    */
    //void GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const double dSDFdx, const double dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, PoseDerivs &pd);

    //bool HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const ;

    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    * @param[in] step_id The number of steps done. Used for scaling down the step size.
    */
    //void ScaleJacobian(PoseDerivs &jacobian, const size_t step_number, const size_t pixel_count) const;


    inline double DeltaFunction(double x,const double std){
      //return (1.0f / float(M_PI)) * (1 / (x * x + 1.0f) + float(1e-3));
      //double std = 2.5; // ----0.05
      return (1.0/(std*sqrt(2*M_PI)))*exp(-((x*x)/(2*std*std)));
    }

    inline void SetBlurringScaleFactor(const int image_width){
      BLUR_WIDTH = 0.3 + (0.7 * image_width / 1900);
    }

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in]    jacobian The pose update of the target object.
    */    
    //void ApplyGradientDescentStep(PoseDerivs &jacobian, Pose &pose, const size_t step,  const size_t pixel_count);

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    */
    double GetRegionAgreement(const int r, const int c, const double sdf) ;


    /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    * @return bool The success of the intersection test.
    */
    bool GetTargetIntersections(const int r, const int c, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const ;

    bool GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const;

    boost::shared_ptr<sv::Frame> frame_;
    ci::gl::Fbo front_depth_framebuffer_; /**< Framebuffer to write the front depth values into. */
    ci::gl::Fbo back_depth_framebuffer_; /**< Framebuffer to write the back depth values into. Has 2 colour buffers. */

    ci::gl::GlslProg front_depth_;
    ci::gl::GlslProg back_depth_and_contour_;

    std::string DEBUG_DIR_;

    size_t NUM_STEPS;
    double BLUR_WIDTH;
    const double k_delta_function_std_;
    const double k_heaviside_width_;

  };




}


#endif
