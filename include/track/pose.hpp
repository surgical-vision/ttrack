#ifndef __POSE_HPP__
#define __POSE_HPP__

#include <cinder/Matrix44.h>
#include <cinder/Vector.h>

#include "../../deps/quaternion/inc/quaternion.hpp"


namespace ttrk {

  /**
  * @class Pose
  * @brief Representation of pose for a rigid body.
  * Contains a SE3 transformation
  */
  class Pose {

  public:

    /**
    * Construct an empty pose. Rotation is set to identity and translation to zero vector.
    */
    Pose() {}

    /**
    * Default construction of pose from rotation and translation from base coordinate system.
    * @param[in] rotation The rotation as a quaternion.
    * @param[in] translation The translation as a 3-vector.
    */

    Pose(const sv::Quaternion &rotation, const ci::Vec3f &translation) : rotation_(rotation), translation_(translation) {}

    Pose(const ci::Matrix44f &t);

    std::vector<ci::Vec3f> ComputeJacobian(const ci::Vec3f &point) const;

    operator ci::Matrix44f() const;

  protected:

    ci::Vec3f translation_; /**< The translation vector. */
    sv::Quaternion rotation_; /**< The rotation matrix. */

  };

}


//namespace ttrk {
//
//  struct PoseDerivs {
//    
//    void AddValues(const PoseDerivs &pd) { for(int i=0;i<NUM_VALS;++i) vals_[i] += pd.vals_[i]; }
//    void MultiplyByValue(const double region_agreement) { for(int i=0;i<NUM_VALS;++i) vals_[i] *= region_agreement; }
//    const static int NUM_VALS;    
//    double operator[](const int n) const { return vals_[n]; }
//    double &operator[](const int n) { return vals_[n]; }
//    PoseDerivs() : vals_(new double[NUM_VALS]) {}
//    ~PoseDerivs() { delete vals_; }
//    static PoseDerivs Zeros() { PoseDerivs pd; memset(pd.vals_,0,sizeof(double)*NUM_VALS); return pd; }
//    double *vals_;
//   
//  };
//  
// 
//  struct Pose {
//
//    inline Pose():rotation_(1,cv::Vec3d(0,0,0)),translation_(0,0,0){}
//
//    inline Pose(const cv::Vec3d &v, const cv::Vec4f &r): translation_(v), rotation_(r[0],cv::Vec3d(r[1],r[2],r[3])){}
//
//    inline Pose(const cv::Vec3d &v, const sv::Quaternion &r):translation_(v),rotation_(r){}
//
//    Pose operator=(const cv::Mat &that);
//
//    /**
//    * Transform the pose from our left handed coordinate system to an OpenGL/Cinder friendly one.
//    * This means column-major storage and right handed coordinates system with camera looking down -z.
//    * @return A transformed matrix.
//    */
//    ci::Matrix44d AsCiMatrixForOpenGL() const ;
//
//    inline Pose(const Pose &that){
//      translation_ = that.translation_;
//      rotation_ = that.rotation_;
//      translational_velocity_ = that.translational_velocity_;
//    }
//
//    inline Pose &operator=(const Pose &rhs){
//      Pose tmp(rhs);
//      std::swap(translation_,tmp.translation_);
//      std::swap(rotation_,tmp.rotation_);
//      std::swap(translational_velocity_,tmp.translational_velocity_);
//      return *this;
//    }
//
//
//    inline cv::Vec3d Transform(const cv::Vec3d &to_transform) const {
//      return rotation_.RotateVector(to_transform) + translation_;
//    }
//
//    inline cv::Vec3d InverseTransform(const cv::Vec3d &to_inv_transform) const {
//      cv::Vec3d t = to_inv_transform - translation_;
//      sv::Quaternion inverse_rotation = rotation_.Inverse();
//      return inverse_rotation.RotateVector(t);
//    }
//
//    inline Pose Inverse() const {
//
//      Pose p;
//      p.rotation_ = rotation_.Inverse();
//      p.translation_ = -p.rotation_.RotateVector(translation_);
//      return p;
//    
//    }
//
//    cv::Vec3d GetDOFDerivatives(const int dof, const cv::Vec3d &point) const ;
//    void GetFastDOFDerivs(double *data, double *point) const;
//    void SetupFastDOFDerivs(double *data) const;
//
//    operator cv::Mat() const;
//
//    cv::Vec3d translation_;
//    sv::Quaternion rotation_;
//
//    cv::Vec3d translational_velocity_;
//    //cv::Vec3d rotational_velocity;
//
//  };
//
//  inline bool operator==(const Pose &p1, const Pose &p2){
//    return p1.translation_ == p2.translation_ && p1.rotation_ == p2.rotation_;
//  }
//
//  inline bool operator!=(const Pose &p1, const Pose &p2){
//    return !(p1==p2);
//  }
//
//
//  inline std::ostream &operator<<(std::ostream &os, const Pose &p){
//    os << "[" << p.rotation_ << ", " << p.translation_[0] << ", " << p.translation_[1] << ", " << p.translation_[2] << "]";
//    return os;
//  }
//
//  inline Pose CombinePoses(const Pose &a, const Pose &b) {
//
//    Pose p;
//    p.translation_ = a.translation_ + a.rotation_.RotateVector(b.translation_);
//    p.rotation_ = a.rotation_ * b.rotation_;
//    return p;
//
//  }
// }

#endif
