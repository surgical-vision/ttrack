#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

#include <cv.h>
#include <boost/shared_ptr.hpp>
#include "model/model.hpp"
#include "pose.hpp"
#include <fstream>

namespace ttrk {

 /**
  * @struct KalmanTracker
  * @brief A container to hold a tracked model and the kalman filter associated with it.
  */
  class KalmanTracker {

  public:
    
    //KalmanTracker() : save_file_(new std::ofstream), model_(new Model {}
    KalmanTracker(boost::shared_ptr<Model> model):save_file_(new std::ofstream),model_(model),filter_(14,7,0,CV_32F){}   
    KalmanTracker(const KalmanTracker &that);

    void SetPose(const cv::Vec3d translation, const cv::Vec3d rotated_principal_axis); 
    void SetPose(const Pose &p);
    void UpdatePose(const Pose &pose_measurement) ;
    void Init();
    /**
    * Accessor to the pose. This can be stored in whichever form the user chooses. For instance, an \f$\mathcal{SE}3\f$ transformation  or a 7x1 vector of a quaternion and a position.
    * @return A reference to the pose.
    */
    Pose &CurrentPose() { return pose_; }
    const Pose &CurrentPose() const { return pose_; }
    boost::shared_ptr<Model> PtrToModel(){ return model_; }
    const boost::shared_ptr<Model> PtrToModel() const { return model_; }
    //boost::shared_ptr<std::vector<Object *> > ModelPointsAtCurrentPose() const { return model_->Points(pose_); }

    boost::shared_ptr<std::ofstream> SaveFile() { return save_file_; }

  protected:

    boost::shared_ptr<std::ofstream> save_file_; /**< A file handle where the pose is saved for results logging. */
    
    cv::KalmanFilter filter_; /**< The Kalman Filter used to track the class. */
    boost::shared_ptr<Model> model_; /**< The tracked model. */
    cv::Mat temporary_update_pose; /**< A store for the pose update. May be removed. */

    Pose pose_; /**< The pose of the model. */

  private:

    KalmanTracker() : save_file_(new std::ofstream) {}

  };



}

#endif
