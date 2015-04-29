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

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(frame_->GetClassificationMap(),connected_regions)) {
    return false;
  }

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    TemporalTrackedModel new_tracker;
    tracked_models_.push_back(new_tracker);

    tracked_models_.back().model.reset(new DenavitHartenbergArticulatedModel(model_parameter_file_, results_dir_ + "/tracked_model" + Model::GetCurrentModelCount() + ".txt"));
    tracked_models_.back().temporal_tracker.reset(new KalmanFilterTracker);

    InitFromFile(tracked_models_.back().model);

  }

  return true;

}


void MonocularToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region, boost::shared_ptr<Model> tracked_model){

  cv::Vec2d center_of_mass = FindCenterOfMass(connected_region);
  cv::Mat moi_tensor = cv::Mat::zeros(2,2,CV_32FC1);
  float *data = (float *)moi_tensor.data;
  
  /* moi [ xx , xy ; yx , yy ] */
  for(int r=0;r<2;r++){
    for(int c=0;c<2;c++){
      if(r==1 && c== 0) {
	      //symmetric...
	      data[r*2 + c] = data[c*2 + r];
	      continue;
      }
      
      for(size_t i=0;i<connected_region.size();i++){

        cv::Vec2d p = cv::Vec2d(connected_region[i]) - center_of_mass;
        int p_i = (int)(p[0]*(1-r) + p[1]*r);
        int p_j = (int)(p[0]*(1-c) + p[1]*c);

        data[r*2 + c] += (float)(( (p[0]*p[0]+p[1]*p[1])*(r==c)) 
          - (p_i*p_j) );
      }
    }
  }

  cv::Mat eigenvals = cv::Mat::zeros(2,1,CV_32FC1);
  cv::Mat eigenvecs = cv::Mat::zeros(2,2,CV_32FC1);
  cv::eigen(moi_tensor,eigenvals,eigenvecs);

  float *e = (float *)eigenvecs.data;
  float *v = (float *)eigenvals.data;
  
  cv::Vec2d central_axis(e[2],e[3]);
  cv::Vec2d horizontal_axis(e[0],e[1]);

  CheckCentralAxisDirection(center_of_mass,central_axis);
  
  cv::Vec2d normed_central_axis,normed_horizontal_axis;
  cv::normalize(central_axis,normed_central_axis);
  cv::normalize(horizontal_axis,normed_horizontal_axis);
  central_axis = normed_central_axis;
  horizontal_axis = normed_horizontal_axis; 

  if(v[1] < v[0]){
    float tmp = v[1];
    v[1] = v[0];
    v[0] = tmp;
  }

  const double radius = sqrt( (2.0*std::abs(v[0]))/connected_region.size() ); 
  const double length = sqrt( ((12.0*std::abs(v[1])) / connected_region.size())  - 3*radius*radius);
  
  cv::Vec2d point = cv::Vec2d(center_of_mass) + 0.5*length*central_axis;
  cv::Vec2d top = cv::Vec2d(center_of_mass) + (radius)*horizontal_axis;
  cv::Vec2d bottom = cv::Vec2d(center_of_mass) - (radius)*horizontal_axis;

  cv::Point3d top_unp = camera_->UnProjectPoint(cv::Point2i(top));
  cv::Point3d bottom_unp = camera_->UnProjectPoint(cv::Point2i(bottom));
  cv::Point3d center_unp = camera_->UnProjectPoint(cv::Point2i(center_of_mass));
  cv::Vec3d diff = cv::Vec3d(top_unp) - cv::Vec3d(bottom_unp);
  double abs_diff = sqrt( static_cast<double>( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] ) );

  
  double z = 80;

  ci::Vec3f trans(center_unp.x, center_unp.y, center_unp.z);
  trans *= z;

  tracked_model->SetBasePose(Pose(ci::Quatf(0, 0, central_axis.dot(cv::Vec2f(0, 1))), trans));
  
} 

float MonocularToolTracker::ComputeWidth(float e1, float e2, size_t mass) const {
  float i_x = std::abs(e1);
  float i_y = std::abs(e2);
  
  if(i_x < i_y)
    std::swap(i_x,i_y);
  
  return (float)sqrt( (2.0*i_y) / mass );

}

const cv::Vec2i MonocularToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2d com(0,0);

  for(auto pt = connected_region.begin() ; pt != connected_region.end() ; pt++ ){

    com += *pt;

  }
  
  com[0] = com[0]/connected_region.size();
  com[1] = com[1]/connected_region.size();

  return com;

}





