#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "comp_ls.hpp"
#include "../../model/node.hpp"

namespace ttrk {

  class ArticulatedComponentLevelSet : public ComponentLevelSet {

  public:

    ArticulatedComponentLevelSet(size_t number_of_articulated_components, size_t number_of_level_set_components, boost::shared_ptr<StereoCamera> camera) : ComponentLevelSet(number_of_level_set_components, camera), number_of_articulated_components_(number_of_articulated_components), current_pose_param_(0), num_rigid_steps_(40), current_rigid_step_(0), previous_error_value_(-1.0f){}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);   

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &frame_idx_image);

   protected:

    float DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points);

    void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 4, 1> &articulated_jacobian, float &error);

    void UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_image, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian);
    void UpdateArticulatedJacobianRightEye(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian);
    /**
    * Get the error value for the current image.
    * @param[in] classification_image The classification image for this frame.
    * @param[in] row_idx The x-pixel coordinate for this image.
    * @param[in] col_idx The y-pixel coordinate for this image.
    * @param[in] sdf_value The signed distance function value for this pixel.
    * @param[in] target_label The target label for multiclass classification.
    * @param[in] articulated_component_index The index of the articulated component map.
    * @return The error value.
    */
    float GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const size_t target_probability, const size_t neighbour_probability, const size_t articulated_component_index) const;


    struct ComponentData {

      cv::Mat sdf_image_;
      cv::Mat front_intersection_image_;
      cv::Mat back_intersection_image_;
      
      cv::Mat sdf_;

      Node *model_;

      //body (roll) -> head (w.p) -> no-geom (w.y) -> no-geom (clasper-base) -> 

      //component data should mirror structure of Model-tree

      //SDF for this component

      //index of the component in the model (should this be a Model(Node) thing)

      //jacobian could

      //detection color/label mapping

    };

    const size_t number_of_articulated_components_;

    boost::shared_ptr<StereoCamera> stereo_camera_;

    int current_pose_param_;

    int num_rigid_steps_;
    
    int current_rigid_step_; 

    float previous_error_value_;

  };





}


#endif
