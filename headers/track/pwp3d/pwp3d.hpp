#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

#include "../localizer.hpp"
#include "../../utils/camera.hpp"
#include <image/image.hpp>

namespace ttrk {

  class PWP3D : public Localizer {
  public: 
    
    virtual cv::Mat TrackTargetInFrame(KalmanTracker &model) = 0;

  protected:
      
    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    */
    void ScaleJacobian(cv::Mat &jacobian) const;

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in] jacobian The pose update of the target object.
    */    
    void ApplyGradientDescentStep(const cv::Mat &jacobian);

    /**
    * Experimental! Finds and sets a ROI image around the target object. This is to reduce the tracking cost of computing the value of the energy function on pixels which are far from the object contour.
    * @param[in] convex_hull The convex hull of the points which make up the shape.
    */
    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull) = 0;
  
    boost::shared_ptr<tcv::Image<unsigned char,3> > frame_;
    
    
  };


  class MonoPWP3D : public PWP3D {

  public: 
    
    virtual cv::Mat TrackTargetInFrame(KalmanTracker &model);
   

  protected:

     virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull);

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    * @param[in] norm_foreground The normalization constant for the foreground class.
    * @param[in] norm_background The normalization constnat for the background class.
    */
    double GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const;

    /**
    * Get the second part of the derivative. The derivative of the contour w.r.t the pose parameters.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] dSDFdx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dSDFdy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel.
    * @param[in[ sdf The current value of the signed distance function at the pixel /f$(r,c)/f$.
    * @return The pose derivitives as a vector.
    */
    cv::Mat GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf, KalmanTracker &current_model);
    
    /**
    * Construct a signed distance function of the outer contour of the shape projected into the image plane.
    * @param[in] current_model The model which will be projected to the image plane.
    * @return The image containin the signed distance function. Will be a single channel floating point image.
    */
    const cv::Mat ProjectShapeToSDF(KalmanTracker &current_model);

    /**
    * Computes the normalization constant of the pose update equation.
    * @param[out] norm_foreground The normalization constant for the foreground class.
    * @param[out] norm_background The normalization constant for the background class.
    * @param[in] sdf_image The signed distance function image.
    */
    void ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const;

      /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    */
    void GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, KalmanTracker &current_model);
  
  protected:

    boost::shared_ptr<MonocularCamera> camera_;
    cv::Mat ROI; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
  };

  class StereoPWP3D : public PWP3D {
    
  public: 
    virtual cv::Mat TrackTargetInFrame(KalmanTracker &model){
      cv::Mat x;
      return x;

    }

  protected:

    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull);

    boost::shared_ptr<StereoCamera> camera_;
    cv::Mat ROI_left_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    cv::Mat ROI_right_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };


}


#endif
