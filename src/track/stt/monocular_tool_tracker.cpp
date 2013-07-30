#include "../../../headers/track/stt/monocular_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"

using namespace ttrk;

MonocularToolTracker::MonocularToolTracker(const float radius, const float height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_(calibration_filename){
  localizer_.reset(new MonoPWP3D);
}

bool MonocularToolTracker::Init(){

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions((frame_->Mat()),connected_regions)) return false;

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    KalmanTracker new_tracker(boost::shared_ptr<Model>(new  MISTool(radius_,height_) ));
    //new_tracker.model_.reset( new MISTool(radius_,height_) );

    tracked_models_.push_back( new_tracker ); 

    Init2DPoseFromMOITensor(*connected_region);

  }

  return true;

}


void MonocularToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region){

  const cv::Vec2i center_of_mass = FindCenterOfMass(connected_region);

}

const cv::Vec2i MonocularToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2i ret(0,0);
  return ret;

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





