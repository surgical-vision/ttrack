#ifndef __PWP3D_HPP__
#define __PWD3D_HPP__

#include "../localizer.hpp"
#include "../../utils/camera.hpp"
#include <image/image.hpp>
#include "../pose.hpp"

namespace ttrk {

  class PWP3D : public Localizer {
  public: 
    
    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame) = 0;

  protected:
      
    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    */
    void ScaleJacobian(cv::Mat &jacobian) const;

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
    void ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose);

    /**
    * Experimental! Finds and sets a ROI image around the target object. This is to reduce the tracking cost of computing the value of the energy function on pixels which are far from the object contour.
    * @param[in] convex_hull The convex hull of the points which make up the shape.
    */
    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull) = 0;
  
    boost::shared_ptr<sv::Frame> frame_;
    
    
  };


  
  
}


#endif
