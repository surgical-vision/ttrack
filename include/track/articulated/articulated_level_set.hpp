#ifndef __ARTICULATED_LEVEL_SET_HPP__
#define __ARTICULATED_LEVEL_SET_HPP__

#include "../localizer.hpp"
#include "../pwp3d/stereo_pwp3d.hpp"
#include "../model/node.hpp"
#include <ceres/ceres.h>

namespace ttrk {

#define PRECISION 11 //7 + WP + WY + C1 + C2
#define NUM_RESIDUALS 7000


  //struct LevelSetResidual {

  //  LevelSetResidual(int r_idx, int c_idx) : r_(r_idx), c_(c_idx) { }

  //  template <typename T>
  //  bool operator()(const T* const pose,
  //    const T* const point,
  //    T* residuals) const {
  //     
  //    if ()
  //    int x = point

  //    return true;

  //  }

  //  template<typename T>
  //  bool NeedsUpdate(const T* const pose) const {

  //    for (int i = 0; i < current_pose.size(); ++i){
  //      if (std::abs(pose[i] - current_pose[i]) > std::numeric_limits<double>::epsilon()) return true;
  //    }

  //    return false;

  //  }

  //  template<typename T>
  //  void Update(const T* const pose){

  //    GLOBAL_ALS->SetParameters(pose);

  //    GLOBAL_ALS->

  //  }

  //  int r_;
  //  int c_;

  //  static std::vector<double> current_pose;

  //};



  //class ArticulatedLevelSetSolverSingleResidual : public Localizer, public ceres::AutoDiffCostFunction<LevelSetResidual, NUM_RESIDUALS, 4, 3, 5> {

  //public:
  //  
  //  ArticulatedLevelSetSolverSingleResidual(boost::shared_ptr<StereoCamera> camera, boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) : stereo_camera_(camera), current_model_(model), frame_(frame) {}

  //  virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  //  virtual bool NeedsNewFrame() const { return true; }

  //protected:

  //  boost::shared_ptr<Model> current_model_;

  //  boost::shared_ptr<StereoCamera> stereo_camera_;
  //  boost::shared_ptr<sv::Frame> frame_; /**< Just a reference to the current frame, probably not really useful, may be removed. */

  //};

  //public:

  //  ArticulatedLevelSetSolverSingleResidual(const int row_idx, const float col_idx, bool is_first);

  //  virtual bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const;
  //  
  //protected:

  //  bool is_first_residual_so_refresh_model_;

  //};

  class ArticulatedLevelSetSolver : public Localizer, public ceres::SizedCostFunction<NUM_RESIDUALS, 3, 4, 5> {
  
  public:

    ArticulatedLevelSetSolver(boost::shared_ptr<StereoCamera> camera, boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) : stereo_camera_(camera), current_model_(model), frame_(frame) {}

    ArticulatedLevelSetSolver(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera) {}

    virtual bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const;

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

    virtual bool NeedsNewFrame() const { return true; }

  protected:
    
    std::vector< std::pair<int, int> > row_col_residual_idx_;

    boost::shared_ptr<Model> current_model_;

    boost::shared_ptr<StereoCamera> stereo_camera_;
    boost::shared_ptr<sv::Frame> frame_; /**< Just a reference to the current frame, probably not really useful, may be removed. */
    
  };

  class ArticulatedLevelSet : public PWP3D {

  public:

    ArticulatedLevelSet(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera) , PWP3D(camera->left_eye()->Width(), camera->left_eye()->Height()) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);   

    void ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &frame_idx_image);

    float GetErrorValue(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const;

    void SetParameters(boost::shared_ptr<Model> current_model, double const* const*parameters);

    cv::Matx<float, PRECISION, 1> ComputeJacobiansSummed(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image);

    cv::Matx<float, NUM_RESIDUALS, PRECISION> ComputeJacobians(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image);


  protected:

    //end ceres stuff
    /**
    * Get the region agreement for the pixel in a multiclass labelling setup. The multiclass classification in binarized in a one-vs-all strategy.
    * @param[in] row_idx The row index for the classification image.
    * @param[in] col_idx The col index for the classification image.
    * @param[in] sdf_value The signed distance function value for the current pixel.
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


  };





}


#endif
