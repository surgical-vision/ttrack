#include "../../../headers/track/stt/monocular_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"

using namespace ttrk;

MonocularToolTracker::MonocularToolTracker(const float radius, const float height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_(calibration_filename){
  localizer_.reset(new MonoPWP3D);
}

bool MonocularToolTracker::Init(){

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(*(frame_->PtrToClassificationMap()),connected_regions)) return false;

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    KalmanTracker new_tracker(boost::shared_ptr<Model>(new  MISTool(radius_,height_) ));
    //new_tracker.model_.reset( new MISTool(radius_,height_) );

    tracked_models_.push_back( new_tracker ); 

    Init2DPoseFromMOITensor(*connected_region,tracked_models_.back());

  }

  return true;

}


void MonocularToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region, KalmanTracker &tracked_model){

  const cv::Vec2i center_of_mass = FindCenterOfMass(connected_region);
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

        cv::Vec2i p = connected_region[i] - center_of_mass;
        int p_i = p[0]*(1-r) + p[1]*r;
        int p_j = p[0]*(1-c) + p[1]*c;

        data[r*2 + c] +=  (( (p[0]*p[0]+p[1]*p[1])*(r==c)) 
          - (p_i*p_j) );
      }
    }
  }

  cv::Mat eigenvals = cv::Mat::zeros(2,1,CV_32FC1);
  cv::Mat eigenvecs = cv::Mat::zeros(2,2,CV_32FC1);
  cv::eigen(moi_tensor,eigenvals,eigenvecs);

  float *e = (float *)eigenvecs.data;
  float *v = (float *)eigenvals.data;

  float width = ComputeWidth(v[0],v[1],connected_region.size());



  cv::Vec2f central_axis(e[0],e[1]);
  cv::Vec2f horizontal_axis(e[2],e[3]);

  cv::Vec2f top = cv::Vec2f(center_of_mass) + (0.5*width)*horizontal_axis;
  cv::Vec2f bottom = cv::Vec2f(center_of_mass) - (0.5*width)*horizontal_axis;

  cv::Point3f top_unp = camera_.UnProjectPoint(cv::Point2i(top));
  cv::Point3f bottom_unp = camera_.UnProjectPoint(cv::Point2i(bottom));
  
  cv::Vec3f diff = cv::Vec3f(top_unp) - cv::Vec3f(bottom_unp);
  float abs_diff = sqrt( static_cast<double>( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] ) );
  
  float z = tracked_model.PtrToModel()->Radius()/abs_diff;

  cv::Vec3f center = z*cv::Vec3f(center_of_mass[0],center_of_mass[1],1);
  
  tracked_model.SetPose(center,cv::Vec3f(central_axis[0],central_axis[1],0));

}

float MonocularToolTracker::ComputeWidth(float e1, float e2, size_t mass) const {
  float i_x = std::abs(e1);
  float i_y = std::abs(e2);
  
  if(i_x < i_y)
    std::swap(i_x,i_y);
  
  return sqrt( (2.0*i_y) / mass );

}

const cv::Vec2i MonocularToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2f com(0,0);

  for(auto pt = connected_region.begin() ; pt != connected_region.end() ; pt++ ){

    com += *pt;

  }
  
  com[0] = com[0]/connected_region.size();
  com[1] = com[1]/connected_region.size();

  return com;

}

void MonocularToolTracker::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

  std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();
  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){
    throw(std::runtime_error("Error, finish this function!\n"));
//  cv::Point cvpt(tpt[0],tpt[1]);
   /* for(size_t n=0;n<pt.neighbours_.size();n++){

      ttk::Point &npt = (*transformed_points)[pt.neighbours_[n]];
      TooN::Vector<2> tnpt = ProjectPoint(npt.vertex_);
      cv::Point cvnpt(tnpt[0],tnpt[1]);

      if(image.channels() == 3)
	line(image,cvpt,cvnpt,cv::Scalar(255,0,255),3,8);
      if(image.channels() == 1)
	line(image,cvpt,cvnpt,(uchar)255,3,8);
    }
  }
  */
  }

}





