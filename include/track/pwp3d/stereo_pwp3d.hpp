#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {

  enum CameraEye { LEFT = 0, RIGHT = 1 , INV = 2};
  inline CameraEye operator++(CameraEye &e, int){
    const CameraEye eye = e;
    const int i = static_cast<int>(e);
    e = static_cast<CameraEye>(i+1);
    return eye;
  }
  
  class StereoPWP3D : public PWP3D {

  public: 

    StereoPWP3D(boost::shared_ptr<StereoCamera> camera) : PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()), stereo_camera_(camera) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);
    
    virtual float GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf, size_t fg_size, size_t bg_size){

      const float pixel_probability = classification_image.at<float>(r, c);
      const float heaviside_value = HeavisideFunction(sdf);

      const float Pf = pixel_probability / (fg_size * pixel_probability + bg_size * (1 - pixel_probability));
      const float Pb = (1 - pixel_probability) / (fg_size * pixel_probability + bg_size * (1 - pixel_probability));

      return (Pf - Pb) / ((heaviside_value*Pf) + ((1 - heaviside_value)*Pb));

    }

    virtual bool HasConverged() const { 
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

  protected:   
    
    void clearup(){
      ci::app::console() << "errors = [";
      for (auto i = 0; i < errors_.size(); ++i){
        ci::app::console() << errors_[i] << ", ";
      }
      ci::app::console() << "]\n" << std::endl;
      errors_.clear();
    }

    void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error);

    void UpdateJacobianRightEye(const float region_agreement, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, 7> &jacobian);

    //void GetRenderedModelAtPose(const boost::shared_ptr<Model> current_model, cv::Mat &left_canvas, cv::Mat &left_z_buffer, cv::Mat &left_binary_image, cv::Mat &right_canvas, cv::Mat &right_z_buffer, cv::Mat &right_binary_image) const;
    //virtual void GetFastDOFDerivs(const Pose &pose, double *pose_derivs, double *intersection);
    //void GetFastDOFDerivsLeft(const Pose &pose, double *pose_derivs, double *intersection);
    //void GetFastDOFDerivsRight(const Pose &pose, double *pose_derivs, double *intersection);
    
    //bool SwapEye(Pose &pose);
    //void SwapToRight(Pose &pose);
    //void SwapToLeft(Pose &pose);

    CameraEye current_eye_;
    boost::shared_ptr<StereoCamera> stereo_camera_;

    std::vector<float> errors_;

  };


}

#endif
