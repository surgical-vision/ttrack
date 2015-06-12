#include "../../../include/ttrack/track/tracker/monocular_tool_tracker.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/mono_pwp3d.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"
#include "../../../include/ttrack/track/model/articulated_model.hpp"

using namespace ttrk;

MonocularToolTracker::MonocularToolTracker(const std::string &model_parameter_file, const std::string &calibration_filename, const std::string &results_dir, const LocalizerType &localizer_type, const size_t number_of_labels) :SurgicalToolTracker(model_parameter_file, results_dir), camera_(new MonocularCamera(calibration_filename)){
  
  if (localizer_type == LocalizerType::PWP3DLocalizer)
    localizer_.reset(new MonoPWP3D(camera_));
  else
    throw std::runtime_error("");

}

bool MonocularToolTracker::Init(){

  for (auto i = 0; i < Model::GetCurrentModelCountInt(); ++i){

    TemporalTrackedModel new_tracker;
    tracked_models_.push_back(new_tracker);

    std::stringstream ss; ss << i;
    tracked_models_.back().model.reset(new DenavitHartenbergArticulatedModel(model_parameter_file_, results_dir_ + "/tracked_model" + ss.str() + ".txt"));
    tracked_models_.back().temporal_tracker.reset(new KalmanFilterTracker);

    InitFromFile(tracked_models_.back().model, i);

  }

  return true;

}






