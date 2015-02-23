#include <fstream>
#include <stdlib.h>
#include <time.h> 
#include <stdint.h>
#include <boost/timer.hpp>
#include <cinder/app/App.h>
#include <CinderOpenCV.h>

#include "../../../include/ttrack/track/stt/stereo_tool_tracker.hpp"
#include "../../../include/ttrack/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../include/ttrack/track/pwp3d/comp_ls.hpp"
#include "../../../include/ttrack/track/articulated/articulated_level_set.hpp"

#include "../../../include/ttrack/track/model/articulated_model.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

StereoToolTracker::StereoToolTracker(const std::string &model_parameter_file, const std::string &calibration_file, const std::string &results_dir, const LocalizerType &localizer_type) : SurgicalToolTracker(model_parameter_file, results_dir), camera_(new StereoCamera(calibration_file)){

  if (localizer_type == LocalizerType::PWP3DLocalizer)
    localizer_.reset(new StereoPWP3D(camera_));
  else if (localizer_type == LocalizerType::ArticulatedLevelSetLocalizer)
   localizer_.reset(new ArticulatedLevelSet(camera_));
  else if (localizer_type == LocalizerType::ComponentLS)
    localizer_.reset(new ComponentLevelSet(camera_));
  else
    throw std::runtime_error("");	

}



bool StereoToolTracker::Init() {

   boost::shared_ptr<sv::StereoFrame> stereo_frame_ = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //find the connected regions in the image
  std::vector<std::vector<cv::Vec2i> >connected_regions;

  //get a foreground background segmentation 
  cv::Mat binary_classification_image = 255 * stereo_frame_->GetBinaryClassificationMapROI(0);

  if (!FindConnectedRegions(binary_classification_image, connected_regions)) {

    ci::app::console() << "Failing to find connected regions!" << std::endl;
    return false;

  }
  
  //for each connected region find the corresponding connected region in the other frame
  for (auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    TemporalTrackedModel new_tracker;
    tracked_models_.push_back(new_tracker);

    tracked_models_.back().model.reset(new DenavitHartenbergArticulatedModel(model_parameter_file_, results_dir_ + "/tracked_model" + Model::GetCurrentModelCount() + ".txt"));
    tracked_models_.back().temporal_tracker.reset(new KalmanFilterTracker);

    InitFromFile(tracked_models_.back().model);

    //JUST A HACK FOR TESTING!!!
    if (tracked_models_.size() > 0) break;

  }

  return true;
}

cv::Vec2d StereoToolTracker::FindCenterOfMassIn2D(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2d com(0, 0);

  for (auto pt = connected_region.begin(); pt != connected_region.end(); pt++){

    com += *pt;

  }

  com[0] = com[0] / connected_region.size();
  com[1] = com[1] / connected_region.size();

  return com;

}


void StereoToolTracker::InitIn2D(const std::vector<cv::Vec2i> &connected_region, cv::Vec3d &center_of_mass_3d, cv::Vec3d &central_axis_3d, boost::shared_ptr<MonocularCamera> camera, boost::shared_ptr<Model> tm) {
  
  /*
	cv::Vec2d center_of_mass = FindCenterOfMassIn2D(connected_region);

	cv::Mat moi_tensor = cv::Mat::zeros(2, 2, CV_32FC1);
	float *data = (float *)moi_tensor.data;

	// moi [ xx , xy ; yx , yy ]
	for (int r = 0; r < 2; r++){
		for (int c = 0; c < 2; c++){
			if (r == 1 && c == 0) {
				//symmetric...
				data[r * 2 + c] = data[c * 2 + r];
				continue;
			}

			for (size_t i = 0; i < connected_region.size(); i++){

				cv::Vec2d p = cv::Vec2d(connected_region[i]) - center_of_mass;
				int p_i = p[0] * (1 - r) + p[1] * r;
				int p_j = p[0] * (1 - c) + p[1] * c;

				data[r * 2 + c] += (((p[0] * p[0] + p[1] * p[1])*(r == c))
					- (p_i*p_j));
			}
		}
	}

	cv::Mat eigenvals = cv::Mat::zeros(2, 1, CV_32FC1);
	cv::Mat eigenvecs = cv::Mat::zeros(2, 2, CV_32FC1);
	cv::eigen(moi_tensor, eigenvals, eigenvecs);

	float *e = (float *)eigenvecs.data;
	float *v = (float *)eigenvals.data;

	cv::Vec2d central_axis(e[2], e[3]);
	cv::Vec2d horizontal_axis(e[0], e[1]);

	CheckCentralAxisDirection(center_of_mass, central_axis);

	cv::Vec2d normed_central_axis, normed_horizontal_axis;
	cv::normalize(central_axis, normed_central_axis);
	cv::normalize(horizontal_axis, normed_horizontal_axis);
	central_axis = normed_central_axis;
	horizontal_axis = normed_horizontal_axis;

	if (v[1] < v[0]){
		float tmp = v[1];
		v[1] = v[0];
		v[0] = tmp;
	}

	const float radius = sqrt((2.0*std::abs(v[0])) / connected_region.size());
	const float length = sqrt(((12.0*std::abs(v[1])) / connected_region.size()) - 3 * radius*radius);

	cv::Vec2d point = cv::Vec2d(center_of_mass) + 0.5*length*central_axis;
	cv::Vec2d top = cv::Vec2d(center_of_mass) + (radius)*horizontal_axis;
	cv::Vec2d bottom = cv::Vec2d(center_of_mass) - (radius)*horizontal_axis;


	cv::Point3d top_unp = camera->UnProjectPoint(cv::Point2i(top));
	cv::Point3d bottom_unp = camera->UnProjectPoint(cv::Point2i(bottom));
	cv::Point3d center_unp = camera->UnProjectPoint(cv::Point2i(center_of_mass));
	cv::Vec3d diff = cv::Vec3d(top_unp) - cv::Vec3d(bottom_unp);
	float abs_diff = sqrt(static_cast<double>(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]));


	float z = 80;// (2*tracked_models_.back().PtrToModel()->Radius())/abs_diff;
	//throw(std::runtime_error("Error, do this!\n"));

	cv::Vec3d unp_point = cv::Vec3d(camera->UnProjectPoint(cv::Point2d(point)));
	center_of_mass_3d = cv::Vec3d(camera->UnProjectPoint(cv::Point2d(center_of_mass)));

	central_axis_3d = (z*unp_point) - (z*center_of_mass_3d);
	cv::Vec3d ca3d_norm; cv::normalize(central_axis_3d, ca3d_norm);
	central_axis_3d = ca3d_norm;
	central_axis_3d[2] = 0.45;
	center_of_mass_3d = t * z;

	tm.SetPose(center_of_mass_3d,central_axis_3d);

	boost::shared_ptr<MISTool> tool = boost::dynamic_pointer_cast<MISTool>(tm.PtrToModel());

	cv::Point2i tip = camera->ProjectPointToPixel(tm.CurrentPose().Transform(tool->GetTrackedPoint()));

	while (!cv::Rect(0, 0, frame_->GetImageROI().cols, frame_->GetImageROI().rows).contains(tip)){
		z *= 1.1;
		center_of_mass_3d = cv::Vec3d(camera->UnProjectPoint(cv::Point2d(center_of_mass))) * z;
		tm.SetPose(center_of_mass_3d, central_axis_3d);
		tip = camera->ProjectPointToPixel(tm.CurrentPose().Transform(tool->GetTrackedPoint()));
	}

	ShiftCenter(center_of_mass, central_axis, 2 * l2_distance(cv::Vec2d(tip.x, tip.y), cv::Vec2d(center_of_mass))); // 2 * to turn center to tip distnace to whole distance
	center_of_mass_3d = cv::Vec3d(camera->UnProjectPoint(cv::Point2d(center_of_mass)));
	center_of_mass_3d = center_of_mass_3d * z;
  */

}

void StereoToolTracker::ShiftToTip(const cv::Vec3d &central_axis, cv::Vec3d &center_of_mass) {//, KalmanTracker &tracked_model){

	//throw(std::runtime_error("Error, not implemented!\n"));
	/*
	KalmanTracker t(boost::shared_ptr<Model>(new MISTool(radius_,height_) ));
	const float length_of_central_axis = sqrt( central_axis[0]*central_axis[0] + central_axis[1]*central_axis[1] + central_axis[2]*central_axis[2] );
	boost::shared_ptr<MISTool> mis_tool = boost::static_pointer_cast<MISTool>(t.PtrToModel());

	const cv::Vec3d original_center_of_mass = center_of_mass;

	float length;
	do{

	t.SetPose(center_of_mass,central_axis);
  cv::Vec3d tip_of_instrument = t.CurrentPose().Transform( cv::Vec3d(-this->height_/2 + this->height_*mis_tool->HeightFraction(),0,0) );

  cv::Vec3d com_to_tip = tip_of_instrument - original_center_of_mass;
  length = sqrt( com_to_tip[0]*com_to_tip[0] + com_to_tip[1]*com_to_tip[1] + com_to_tip[2]*com_to_tip[2] );

  center_of_mass = center_of_mass - 0.05*central_axis;

  }while (length > length_of_central_axis);
  */
}


void StereoToolTracker::Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, boost::shared_ptr<Model> tracked_model) {

  cv::Vec3d left_center_of_mass, left_central_axis, right_center_of_mass, right_central_axis;
  InitIn2D(region, left_center_of_mass, left_central_axis, camera_->left_eye(), tracked_model);

}

cv::Vec3d StereoToolTracker::FindPrincipalAxisFromMOITensor(const cv::Vec3d center_of_mass_, const cv::Mat &point_cloud) const {

  cv::Matx<float, 1, 3> principal_axis(0, 0, 0);
  cv::Matx<float, 1, 3> center_of_mass((float)center_of_mass_[0], (float)center_of_mass_[1], (float)center_of_mass_[2]);
  cv::Matx<float, 3, 3> moi(3, 3, CV_32FC1);
  cv::Matx<float, 3, 3> E = cv::Matx<float, 3, 3>::eye();

  const int rows = point_cloud.rows;
  const int cols = point_cloud.cols;
  const int chans = point_cloud.channels();
  assert(point_cloud.type() == CV_32FC3);

  for (int r = 0; r < rows; r++){
    for (int c = 0; c < cols; c++){

      if (point_cloud.at<cv::Vec3d>(r, c) == cv::Vec3d(0, 0, 0) || point_cloud.at<cv::Vec3d>(r, c)[2] < 0) continue;
      const cv::Matx<float, 1, 3> point = point_cloud.at<cv::Matx<float, 1, 3> >(r, c) - center_of_mass;
      moi = moi + ((point*point.t())(0, 0)*E) - (point.t()*point);

    }
  }

  cv::Mat eigenvectors, eigenvalues;
  cv::eigen(moi, eigenvalues, eigenvectors);

  int row = 1;
  return cv::Vec3d(eigenvectors.at<float>(row, 0), eigenvectors.at<float>(row, 1), eigenvectors.at<float>(row, 2));

}

cv::Vec3d StereoToolTracker::FindCenterOfMass(const cv::Mat &point_cloud) const {

  const int rows = point_cloud.rows;
  const int cols = point_cloud.cols;
  const int chans = point_cloud.channels();
  assert(point_cloud.type() == CV_32FC3);

  cv::Vec3d com(0, 0, 0);
  size_t num_pts = 0;

  std::vector<cv::Vec3d> pts;

  for (int r = 0; r < rows; r++){
    for (int c = 0; c < cols; c++){

      const cv::Vec3d &pt = point_cloud.at<cv::Vec3d>(r, c);

      if (pt != cv::Vec3d(0, 0, 0)){
        com += pt;
        num_pts++;
      }


    }
  }

  com[0] = com[0] / num_pts;
  com[1] = com[1] / num_pts;
  com[2] = com[2] / num_pts;

  return com;

}

void StereoToolTracker::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){

  //first set the handle using the superclass method
  Tracker::SetHandleToFrame(image);
  boost::shared_ptr<sv::StereoFrame> stereo_image = boost::dynamic_pointer_cast<sv::StereoFrame>(image);

}

