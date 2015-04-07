#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include <cinder/app/App.h>

#include "pwp3d.hpp"


namespace ttrk {
 
  /**
  * @class StereoPWP3D
  * @brief A class to do PWP3D type tracking with stereo cameras and point registration. 
  */
  class StereoPWP3D : public PWP3D {

  public: 

    /**
    * Default constructor.
    * @param[in] camera The camera we are using for projection etc.
    */
    StereoPWP3D(boost::shared_ptr<StereoCamera> camera);

    /**
    * Specialization of actual frame pose locationization for stereo.
    * @param[in] model The model we are tracking. Pose is updated inside this loop.
    */
    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);
    
 /*   virtual bool HasConverged() const { 
      if (PWP3D::HasConverged()) {
        auto *th = const_cast<StereoPWP3D *>(this);
        th->clearup();
        return true;
      }
      if (errors_.size() < 4) return false; 
      const size_t size = errors_.size();
      if (errors_[size - 1] > errors_[size - 3]) {
        auto *th = const_cast<StereoPWP3D *>(this);
        th->clearup();
        return true;
      }
      return false;
    }
*/
  protected:   
    
    float DoRegionBasedAlignmentStepForLeftEye(boost::shared_ptr<Model> current_model);
    float DoRegionBasedAlignmentStepForRightEye(boost::shared_ptr<Model> current_model);
    float DoPointBasedAlignmentStepForLeftEye(boost::shared_ptr<Model> current_model);


    void clearup(){
      errors_.clear();
    }

    /**
    * Specialization of the jacobian computation each eye.
    * @param[in] classification_image The classification image to use.
    * @param[in] current_model The current model to use.
    * @param[in] camera The camera model for this eye.
    * @param[out] jacobian The jacobian for this eye.
    * @param[out] hessian_approximation The hessian approximation (if using approximate Newton (~Gauss Newton) optimization).
    * @param[out] error The error for this frame.
    */
    virtual void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error);

    /**
    * Specialization of the jacobian computation for the right eye (the pose parameters we target are w.r.t. the left eye).
    * @param[in] region_agreement The region agreement for this pixel.
    * @param[in] sdf The sdf value.
    * @param[in] dsdf_dx The sdf value.
    * @param[in] dsdf_dy The sdf value.
    * @param[in] fx The x focal length of the camera.
    * @param[in] fy The y focal length of the camera.
    * @param[in] front_intersection_point The front model intersection point (i.e. intersection closest to the camera).
    * @param[in] back_intersection_point The back model intersection point (i.e. intersection furthest from the camera).
    * @param[in] model The model we are tracking.
    * @param[out] jacobian The jacobian we are updating.
    */
    void UpdateJacobianRightEye(const float region_agreement, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, 7> &jacobian);

    boost::shared_ptr<StereoCamera> stereo_camera_; /**< Representation of the camera. */

    std::vector<float> errors_; /**< The current set of errors. */

  };


}

#endif
