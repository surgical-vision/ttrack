#pragma once

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>

#include "articulated_level_set.hpp"
#include "../../../utils/plotter.hpp"

namespace ttrk{

  class ArticulatedLevelSetSolver : public Localizer, public ErrorMetricPlottable {

  public:

    ArticulatedLevelSetSolver(boost::shared_ptr<StereoCamera> camera, boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) : stereo_camera_(camera), current_model_(model), frame_(frame) {}

    ArticulatedLevelSetSolver(boost::shared_ptr<StereoCamera> camera) : stereo_camera_(camera) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

    virtual bool HasConverged() const { return true; }

  protected:


    boost::shared_ptr<Model> current_model_;

    boost::shared_ptr<StereoCamera> stereo_camera_;
    boost::shared_ptr<sv::Frame> frame_; /**< Just a reference to the current frame, probably not really useful, may be removed. */

  };



}