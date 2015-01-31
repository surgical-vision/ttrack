#pragma once

#include <ceres/ceres.h>

#include "articulated_level_set.hpp"

namespace ttrk{



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

    virtual bool HasConverged() const { return true; }

  protected:

    std::vector< std::pair<int, int> > row_col_residual_idx_;

    boost::shared_ptr<Model> current_model_;

    boost::shared_ptr<StereoCamera> stereo_camera_;
    boost::shared_ptr<sv::Frame> frame_; /**< Just a reference to the current frame, probably not really useful, may be removed. */

  };



}