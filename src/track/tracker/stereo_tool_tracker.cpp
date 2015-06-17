#include <fstream>
#include <stdlib.h>
#include <time.h> 
#include <stdint.h>
#include <boost/timer.hpp>
#include <cinder/app/App.h>
#include <CinderOpenCV.h>

#include "../../../include/ttrack/track/tracker/stereo_tool_tracker.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/stereo_pwp3d.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/comp_ls.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/articulated_level_set.hpp"

#include "../../../include/ttrack/track/model/articulated_model.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

#ifdef USE_CERES
#include "../../../include/ttrack/track/localizer/levelsets/articulated_solver.hpp"
#endif

using namespace ttrk;

StereoToolTracker::StereoToolTracker(const std::string &model_parameter_file, const std::string &calibration_file, const std::string &results_dir, const LocalizerType &localizer_type, const size_t number_of_labels) : SurgicalToolTracker(model_parameter_file, results_dir), camera_(new StereoCamera(calibration_file)){

  if (localizer_type == LocalizerType::PWP3DLocalizer)
    localizer_.reset(new StereoPWP3D(camera_));
  else if (localizer_type == LocalizerType::ComponentLS)
    localizer_.reset(new ComponentLevelSet(number_of_labels, camera_));
  else if (localizer_type == LocalizerType::ArticulatedComponentLS)
    localizer_.reset(new ArticulatedComponentLevelSet(11, number_of_labels, camera_));
#ifdef USE_CERES
  else if (localizer_type == LocalizerType::CeresLevelSetSolver)
    localizer_.reset(new CeresLevelSetSolver(camera_));
#endif
  else
    throw std::runtime_error("");	

}





bool StereoToolTracker::Init() {

  if (starting_pose_HACK.size() == 0) return false;

  for (auto i = 0; i < starting_pose_HACK.size(); ++i){

    TemporalTrackedModel new_tracker;
    tracked_models_.push_back(new_tracker);

    std::stringstream ss; ss << i;
    tracked_models_.back().model.reset(new DenavitHartenbergArticulatedModel(model_parameter_file_, results_dir_ + "/tracked_model" + ss.str() + ".txt"));
    tracked_models_.back().temporal_tracker.reset(new KalmanFilterTracker);

    InitFromFile(tracked_models_.back().model, i);

  }

  return true;
}

void StereoToolTracker::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){

  //first set the handle using the superclass method
  Tracker::SetHandleToFrame(image);
  boost::shared_ptr<sv::StereoFrame> stereo_image = boost::dynamic_pointer_cast<sv::StereoFrame>(image);

}
