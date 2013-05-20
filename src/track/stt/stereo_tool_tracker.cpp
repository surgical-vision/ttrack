#include "../../../headers/track/stt/stereo_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"

using namespace ttrk;

StereoToolTracker::StereoToolTracker(const int radius, const int height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_(calibration_filename){

  localizer_.reset(new StereoPWP3D);
  
}

bool StereoToolTracker::Init() {
  return false;

}
