#include "../../../headers/track/stt/monocular_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

MonocularToolTracker::MonocularToolTracker(const std::string &model_parameter_file, const std::string &calibration_filename):SurgicalToolTracker(model_parameter_file),camera_(new MonocularCamera(calibration_filename)){
  localizer_.reset(new MonoPWP3D(camera_));
}

bool MonocularToolTracker::Init(){

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(frame_->GetClassificationMap(),connected_regions)) {
#ifdef SAVEDEBUG_1
    std::cerr << "NO connected regions in the classification image found!\n";
    cv::imwrite("./debug/bad_classification.png",frame_->GetImageROI());
    cv::imwrite("./debug/bad_image.png",frame_->GetClassificationMapROI());
#endif
    return false;
  }

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    KalmanTracker new_tracker( boost::shared_ptr<Model>(new MISTool(model_parameter_file_)) );
    tracked_models_.push_back( new_tracker ); 
    Init2DPoseFromMOITensor(*connected_region,tracked_models_.back());

  }

  return true;

}


void MonocularToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region, KalmanTracker &tracked_model){

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

#ifdef CHECK_INIT
  cv::Mat debug_frame = frame_->GetImageROI().clone();
  cv::circle(debug_frame,cv::Point(point),4,cv::Scalar(255,0,0),2);
  cv::circle(debug_frame,cv::Point(center_of_mass),4,cv::Scalar(0,0,255),2);
  cv::line(debug_frame,cv::Point(point),cv::Point(center_of_mass),cv::Scalar(255,255,0),2);
  cv::imwrite("debug/init_axis.png",debug_frame);
#endif

  cv::Point3d top_unp = camera_->UnProjectPoint(cv::Point2i(top));
  cv::Point3d bottom_unp = camera_->UnProjectPoint(cv::Point2i(bottom));
  cv::Point3d center_unp = camera_->UnProjectPoint(cv::Point2i(center_of_mass));
  cv::Vec3d diff = cv::Vec3d(top_unp) - cv::Vec3d(bottom_unp);
  double abs_diff = sqrt( static_cast<double>( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] ) );

  
  double z = 80;// = (2*tracked_models_.back().PtrToModel()->Radius())/abs_diff;
  
  throw(std::runtime_error("whatever\n"));

  cv::Vec3d unp_point = cv::Vec3d(camera_->UnProjectPoint(cv::Point2d(point)));
  cv::Vec3d center_of_mass_3d = cv::Vec3d(camera_->UnProjectPoint(cv::Point2d(center_of_mass)));


  cv::Vec3d central_axis_3d = (z*unp_point) - (z*center_of_mass_3d);
  cv::Vec3d ca3d_norm; cv::normalize(central_axis_3d,ca3d_norm);
  central_axis_3d = ca3d_norm;
  central_axis_3d[2] = 0.45;
  center_of_mass_3d = center_of_mass_3d * z;

  tracked_model.SetPose(center_of_mass_3d,central_axis_3d);

  boost::shared_ptr<MISTool> tool = boost::dynamic_pointer_cast<MISTool>(tracked_model.PtrToModel());

  cv::Point2i tip = camera_->ProjectPointToPixel( tracked_model.CurrentPose().Transform(tool->GetTrackedPoint()));

  while( !cv::Rect(0,0,frame_->GetImageROI().cols,frame_->GetImageROI().rows).contains(tip) ){
    z *= 1.1;
    center_of_mass_3d = cv::Vec3d(camera_->UnProjectPoint(cv::Point2d(center_of_mass))) * z;
    tracked_model.SetPose(center_of_mass_3d,central_axis_3d);
    tip = camera_->ProjectPointToPixel( tracked_model.CurrentPose().Transform(tool->GetTrackedPoint()));
  }

  ShiftCenter(center_of_mass,central_axis, 2 * l2_distance(cv::Vec2d(tip.x,tip.y),cv::Vec2d(center_of_mass))); // 2 * to turn center to tip distnace to whole distance
  center_of_mass_3d = cv::Vec3d(camera_->UnProjectPoint(cv::Point2d(center_of_mass)));
  center_of_mass_3d = center_of_mass_3d * z;
  tracked_model.SetPose(center_of_mass_3d,central_axis_3d);

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

void MonocularToolTracker::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

boost::shared_ptr<std::vector<Object *> > transformed_points= tracked_model.ModelPointsAtCurrentPose();
  for(auto point = transformed_points->begin(); point != transformed_points->end(); point++ ){

    /*cv::Vec2d projected = camera_->ProjectPoint(point->vertex_);

    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){
      
      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2d projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

      if(canvas.channels() == 3)
        line(canvas,cv::Point2d(projected),cv::Point2d(projected_neighbour),cv::Scalar(255,123,25),1,CV_AA);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2d(projected),cv::Point2d(projected_neighbour),(unsigned char)255,1,CV_AA);
    }
    */
  }
}





