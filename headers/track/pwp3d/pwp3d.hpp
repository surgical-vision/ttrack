#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

#include "../localizer.hpp"
#include "../../utils/camera.hpp"
#include "../../../deps/image/image/image.hpp"
#include "../pose.hpp"
#include "register_points.hpp"
#include "../../track/model/fast_bvh/Object.h"


namespace ttrk {

  class PWP3D : public Localizer {
  public: 

    PWP3D(boost::shared_ptr<MonocularCamera> camera) : camera_(camera), register_points_(camera), k_delta_function_std_(2.5), k_heaviside_width_(0.3) { }

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame) = 0;
    
    boost::shared_ptr<MonocularCamera> &Camera() { return camera_; } //references to shared pointers are nasty, change this!

    bool PWP3D::ModelInFrame( const KalmanTracker &tracked_model, const cv::Mat &detect_image) const;


  protected:

    /**
    * Construct a signed distance function of the outer contour of the shape projected into the image plane.
    * @param[in] current_model The model which will be projected to the image plane.
    * @return The image containin the signed distance function. Will be a single channel floating point image.
    */
    void PWP3D::GetSDFAndIntersectionImage(KalmanTracker &current_model, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);
    //const cv::Mat ProjectShapeToSDF(KalmanTracker &current_model);

    /**
    * Get the second part of the derivative. The derivative of the contour w.r.t the pose parameters.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The value of the signed distance function for (r,c).
    * @param[in] dSDFdx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dSDFdy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel.
    * @return The pose derivitives as a vector.
    */
    cv::Mat GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image);

    cv::Mat AlignObjectToEdges(KalmanTracker &current_model, const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &front_projection_image,cv::Mat &save_image);

    bool HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const ;

    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    * @param[in] step_id The number of steps done. Used for scaling down the step size.
    */
    void ScaleJacobian(cv::Mat &jacobian, const size_t step_number, const size_t pixel_count) const;


    inline double DeltaFunction(double x,const double std){
      //return (1.0f / float(M_PI)) * (1 / (x * x + 1.0f) + float(1e-3));
      //double std = 2.5; // ----0.05
      return (1.0/(std*sqrt(2*M_PI)))*exp(-((x*x)/(2*std*std)));
    }

    inline  void SetBlurringScaleFactor(const int image_width){
      blurring_scale_factor_ = 0.3 + (0.7 * image_width/1900); 
    }


    /*double HeavisideFunction(float x){
    const double a = 0.4; //equates to blur between -25 and 25 ---- 0.3
    double r = 1.0/(exp(-a*x) + 1);
    return r;
    }*/

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in]    jacobian The pose update of the target object.
    */    
    void ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose, const size_t step,  const size_t pixel_count);

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    */
    double GetRegionAgreement(const int r, const int c, const float sdf) ;

    static cv::Vec3d GetDOFDerivatives;

    /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    * @return bool The success of the intersection test.
    */
    bool GetTargetIntersections(const int r, const int c, cv::Vec3d &front_intersection, cv::Vec3d &back_intersection, const KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const ;

    bool GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, cv::Vec3d &front_intersection, cv::Vec3d &back_intersection, const KalmanTracker &current_model , const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const ;

    double GetEnergy(const int r, const int c, const float sdf) const;

    void DrawModelOnFrame(boost::shared_ptr<std::vector<Object *> > transformed_points, cv::Mat canvas);


    boost::shared_ptr<sv::Frame> frame_;

    std::string DEBUG_DIR_;
    boost::shared_ptr<MonocularCamera> camera_;

    PointRegistration register_points_;

    double blurring_scale_factor_;
    const double k_delta_function_std_;
    const double k_heaviside_width_;

  };




}


#endif
