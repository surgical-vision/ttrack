#include <boost/filesystem.hpp>

#include "../../../include/ttrack/track/tracker/surgical_tool_tracker.hpp"
#include "../../../include/ttrack/track/localizer/levelsets/pwp3d.hpp"
#include "../../../include/ttrack/utils/image.hpp"


using namespace ttrk;

SurgicalToolTracker::SurgicalToolTracker(const std::string &model_parameter_file, const std::string &results_dir) : Tracker(model_parameter_file, results_dir) {

  if(!boost::filesystem::exists(boost::filesystem::path(model_parameter_file)))
    throw(std::runtime_error("Error, unable to read model file: " + model_parameter_file + "\n"));

}


void SurgicalToolTracker::InitFromFile(boost::shared_ptr<Model> tm, size_t idx){

  std::vector<float> start_poses = GetStartPose(idx);

  ci::Matrix33f r(start_poses[0], start_poses[4], start_poses[8], start_poses[1], start_poses[5], start_poses[9], start_poses[2], start_poses[6], start_poses[10]);

  Pose ret(ci::Quatf(r), ci::Vec3f(start_poses[3], start_poses[7], start_poses[11]));
  tm->SetBasePose(ret);
  tm->UpdatePose(std::vector<float>({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, start_poses[12], start_poses[13], start_poses[14] / 2, start_poses[14] / 2 }));
}

