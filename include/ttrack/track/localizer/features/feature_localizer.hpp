#pragma once

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../../headers.hpp"
#include "../../model/pose.hpp"
#include "../../model/model.hpp"
#include "../../../utils/camera.hpp"

namespace ttrk {

  class FeatureLocalizer {

  public:

    FeatureLocalizer(boost::shared_ptr<MonocularCamera> camera);

    virtual void TrackLocalPoints(cv::Mat &current_frame, boost::shared_ptr<Model> current_model) = 0;

    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model) = 0;
    virtual void InitializeTracker(cv::Mat &current_frame, const boost::shared_ptr<Model> current_model, const cv::Mat &index_image) = 0;

    virtual bool NeedsReset() const;

    std::vector<float> GetDerivativesForPoints(boost::shared_ptr<Model> current_model, const Pose &pose);

    std::vector<float> GetArticulatedDerivativesForPoints(boost::shared_ptr<Model> current_model, const cv::Mat &articulated_index_image);

    std::vector<float> GetArticulatedPointDerivative(const cv::Vec3f &world_previous, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const boost::shared_ptr<Model> current_model, const size_t articulated_component_idx);

    void UpdatePointsOnArticulatedModelAfterDerivatives(boost::shared_ptr<Model> current_model, const cv::Mat &articulated_index_image);

    void UpdateFrameCount(){
      frame_count_++;
    }

    void SetFrontIntersectionImage(cv::Mat &im, boost::shared_ptr<Model> current_model) {


      current_model->mps.front_intersection_image_ = im.clone();

    }

    std::vector<float> GetIntensityDerivative(const cv::Vec3f &world_previous, const float &dx, const float &dy, const Pose &pose);

    std::vector<float> GetResiduals(boost::shared_ptr<Model> current_model) const;

    cv::Mat GetJacobianMatrix(boost::shared_ptr<Model> current_model, const Pose &pose);

    std::vector<float> GetDerivativesForPointsOnRigidBody(boost::shared_ptr<Model> current_model, const Pose &pose, const cv::Mat &index_image);

    /**
    * Update the frame_point of each each TrackedPoint after we have changed the pose.
    * @param[in] pose The pose of the object after the gradient-based update
    */

    void UpdatePointsOnModelAfterDerivatives(boost::shared_ptr<Model> current_model, const Pose &pose);
    
    float GetPointProjectionError(boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, const cv::Mat &articulated_component_image);

  protected:

    std::vector<float> GetPointDerivative(const cv::Vec3f &world_previous_, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose);

    cv::Mat CreateMask(boost::shared_ptr<Model> current_model);
    
    std::vector<cv::Point2f> GetPointsOnPreviousImage(boost::shared_ptr<Model> current_model);

    boost::shared_ptr<MonocularCamera> camera_;

    float current_error_;

    size_t frame_count_;

  };

}