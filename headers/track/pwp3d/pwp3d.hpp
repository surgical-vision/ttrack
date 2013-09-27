#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

#include "../localizer.hpp"
#include "../../utils/camera.hpp"
#include "../../../deps/image/image/image.hpp"
#include "../pose.hpp"

namespace ttrk {

  class PWP3D : public Localizer {
  public: 
    
    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame) = 0;
    boost::shared_ptr<MonocularCamera> &Camera() { return camera_; }

  protected:
    
    virtual cv::Mat &ROI() = 0;

    /**
    * Construct a signed distance function of the outer contour of the shape projected into the image plane.
    * @param[in] current_model The model which will be projected to the image plane.
    * @return The image containin the signed distance function. Will be a single channel floating point image.
    */
    const cv::Mat ProjectShapeToSDF(KalmanTracker &current_model);

    /**
    * Get the second part of the derivative. The derivative of the contour w.r.t the pose parameters.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The value of the signed distance function for (r,c).
    * @param[in] dSDFdx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dSDFdy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel.
    * @return The pose derivitives as a vector.
    */
    cv::Mat GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model);
    

    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    * @param[in] step_id The number of steps done. Used for scaling down the step size.
    */
    void ScaleJacobian(cv::Mat &jacobian, const int step_number) const;

    double DeltaFunction(float x){
      double std = 0.08; // ----0.05
      return (1.0/(std*sqrt(2*M_PI)))*exp(-((x*x)/(2*std*std)));
    }

    double HeavisideFunction(float x){
      const double a = 0.4; //equates to blur between -25 and 25 ---- 0.3
      double r = 1.0/(exp(-a*x) + 1);
      return r;
    }

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in]    jacobian The pose update of the target object.
    */    
    void ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose, const int step);

    /**
    * Experimental! Finds and sets a ROI image around the target object. This is to reduce the tracking cost of computing the value of the energy function on pixels which are far from the object contour.
    * @param[in] convex_hull The convex hull of the points which make up the shape.
    */
    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull) = 0;

    
     /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    * @param[in] norm_foreground The normalization constant for the foreground class.
    * @param[in] norm_background The normalization constnat for the background class.
    */
    double GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) ;
  
    /**
    * Computes the normalization constant of the pose update equation.
    * @param[out] norm_foreground The normalization constant for the foreground class.
    * @param[out] norm_background The normalization constant for the background class.
    * @param[in] sdf_image The signed distance function image.
    */
    void ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const;
    
    cv::Vec3f GetDOFDerivatives(const int dof, const Pose &pose, const cv::Vec3f &point) const ;
    
      /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    * @return bool The success of the intersection test.
    */
    bool GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model) const ;
    
    bool GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model) const ;
    
    double GetEnergy(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const;

    boost::shared_ptr<sv::Frame> frame_;
    
    std::string DEBUG_DIR_;
    boost::shared_ptr<MonocularCamera> camera_;

  };


  
  
}


#endif
