#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"
#include "../model/node.hpp"

namespace ttrk {

#define PRECISION 11 //7 + WP + WY + C1 + C2
#define NUM_RESIDUALS 7000

  class ArticulatedLevelSet : public PWP3D {

  public:

    ArticulatedLevelSet(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera), PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()), current_pose_param_(0), num_rigid_steps_(40), current_rigid_step_(0), previous_error_value_(-1.0f){}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);   

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &frame_idx_image);

    //float GetErrorValue(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const;

    void SetParameters(boost::shared_ptr<Model> current_model, double const* const*parameters);

    cv::Matx<float, PRECISION, 1> ComputeJacobiansSummed(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image);

    cv::Matx<float, NUM_RESIDUALS, PRECISION> ComputeJacobians(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image);

  protected:

    void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 7, 7> &rigid_hessian_approx, cv::Matx<float, 4, 1> &articulated_jacobian, cv::Matx<float, 4, 4> &articulated_hessian_approx, float &error);

    /**
    * Get the region agreement for the pixel in a multiclass labelling setup. The multiclass classification in binarized in a one-vs-all strategy.
    * @param[in] row_idx The row index for the classification image.
    * @param[in] col_idx The col index for the classification image.
    * @param[in] sdf_value The signed distance function value for the current 
    * @param[in] target_label The target label for the pixel, needed to do one-vs-all classification.
    */
    float GetRegionAgreement(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const;

    void UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_image, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, PRECISION> &jacobian);

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

    boost::shared_ptr<StereoCamera> stereo_camera_;

    int current_pose_param_;

    int num_rigid_steps_;
    
    int current_rigid_step_; 

    float previous_error_value_;

  };





}


#endif
