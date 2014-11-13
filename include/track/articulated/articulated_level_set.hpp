#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"
#include "../model/node.hpp"
#include <ceres/ceres.h>

namespace ttrk {

#define PRECISION 12

  class ArticulatedLevelSet : public PWP3D , public ceres::CostFunction {

  public:

    ArticulatedLevelSet(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera) , PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

    void TrackTargetInFrame__old(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame);

  protected:

    /**
    * Get the region agreement for the pixel in a multiclass labelling setup. The multiclass classification in binarized in a one-vs-all strategy.
    * @param[in] row_idx The row index for the classification image.
    * @param[in] col_idx The col index for the classification image.
    * @param[in] sdf_value The signed distance function value for the current pixel.
    * @param[in] target_label The target label for the pixel, needed to do one-vs-all classification.
    */
    float GetRegionAgreement(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const;

    float GetErrorValue(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const;

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &frame_idx_image);

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


  };





}


#endif
