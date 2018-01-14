#ifndef __LK_TRACKER__
#define __LK_TRACKER__

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../../headers.hpp"
#include "../../model/pose.hpp"
#include "../../../utils/camera.hpp"
#include "../../model/model.hpp"
#include "../../../constants.hpp"
#include "feature_localizer.hpp"

namespace ttrk{

  class LKTracker : public FeatureLocalizer {

  public:

    LKTracker(boost::shared_ptr<MonocularCamera> camera);

    virtual void TrackLocalPoints(cv::Mat &current_frame, boost::shared_ptr<Model> current_model);

    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model);

    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &component_image);

  protected:

    cv::Size win_size_;
    cv::TermCriteria term_crit_;
    
    float current_error_;

  };

  class ArticulatedLKTracker : public LKTracker {

  public:

    ArticulatedLKTracker(boost::shared_ptr<MonocularCamera> camera);

    //virtual void TrackLocalPoints(cv::Mat &current_frame);

    //virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model);

    //virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &component_image);

  protected:


  };


  class LKTracker3D : public LKTracker {

  public:

    LKTracker3D(boost::shared_ptr<MonocularCamera> camera);

    void TrackLocalPoints(cv::Mat &current_frame, const Pose &current_pose, boost::shared_ptr<Model> current_model);

    int current_step_index;

    void swap_points(boost::shared_ptr<Model> current_model) { std::swap(current_model->mps.points_test[0], current_model->mps.points_test[1]); }

    void SpatialDerivatives(const cv::Mat im_t1, cv::Mat &temporal_spatial_derivatives, cv::Mat &spatial_derivatives, const Pose &pose, const cv::Mat front_intersection_image);
  
    std::vector<float> GetUpdateStep(boost::shared_ptr<Model> current_model);

    void SetSpatialDerivatives(boost::shared_ptr<Model> current_model, const Pose &pose);

  protected:

    cv::Size win_size_;
    cv::TermCriteria term_crit_;

    float current_error_;

    cv::Mat temporal_spatial_derivatives;
    cv::Mat spatial_derivatives;


  };

  class ArticulalatedLKTrackerFrameToFrame : public ArticulatedLKTracker {


  public:

    explicit ArticulalatedLKTrackerFrameToFrame(boost::shared_ptr<MonocularCamera> camera) : ArticulatedLKTracker(camera) {}

    void TrackLocalPoints(cv::Mat &current_frame, boost::shared_ptr<Model> current_model, const cv::Mat &component_image);

    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model);

    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &component_image);


  protected:

  };



}

#endif