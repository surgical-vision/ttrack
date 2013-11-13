#include "../../../headers/track/stt/stereo_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include <fstream>
#include <stdlib.h>
#include <time.h> 
#include "../../../headers/utils/quasi_dense_stereo.hpp"
#include <stdint.h>
using namespace ttrk;

#define STEREO_SGBM
#undef QUASI

StereoToolTracker::StereoToolTracker(const float radius, const float height, const std::string &config_dir, const std::string &calibration_file):SurgicalToolTracker(radius,height),camera_( new StereoCamera(config_dir + "/" + calibration_file)){

  localizer_.reset(new StereoPWP3D(config_dir,camera_));
  //boost::shared_ptr<StereoPWP3D> stereo_pwp3d = boost::dynamic_pointer_cast<StereoPWP3D>(localizer_);
  //if(!camera_->IsRectified()) camera_->Rectify(cv::Size(1920,1080));
  //stereo_pwp3d->GetStereoCamera() = camera_; //MOVE THESE TO CONSTRUCTOR
  //stereo_pwp3d->Camera() = stereo_pwp3d->GetStereoCamera()->rectified_left_eye();
}



bool StereoToolTracker::Init() {

  boost::shared_ptr<sv::StereoFrame> stereo_frame_ = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //find the connected regions in the image
  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(stereo_frame_->GetClassificationMapROI(),connected_regions)) {
#if defined(_DEBUG) || defined( DEBUG )
    std::cerr << "Could not find any connected regions!\n";
#endif
    return false;
  }

  //for each connected region find the corresponding connected region in the other frame
  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    KalmanTracker new_tracker(boost::shared_ptr<Model> (new MISTool(radius_,height_) ));
    tracked_models_.push_back( new_tracker ); 
    Init3DPoseFromMOITensor(*connected_region, tracked_models_.back());//,corresponding_connected_region);
  
  }

  return true;
}


void StereoToolTracker::ProcessFrame(){

  //CreateDisparityImage();
  //camera_->ReprojectTo3D(StereoFrame()->GetDisparityMap(),StereoFrame()->GetPointCloud(),std::vector<cv::Vec2i>());

}

const cv::Vec2i StereoToolTracker::FindCenterOfMassIn2D(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2f com(0,0);

  for(auto pt = connected_region.begin() ; pt != connected_region.end() ; pt++ ){

    com += *pt;

  }
  
  com[0] = com[0]/connected_region.size();
  com[1] = com[1]/connected_region.size();

  return com;

}

void StereoToolTracker::InitIn2D(const std::vector<cv::Vec2i> &connected_region, cv::Vec3f &center_of_mass_3d, cv::Vec3f &central_axis_3d, boost::shared_ptr<MonocularCamera> cam) {

  const cv::Vec2i center_of_mass = FindCenterOfMassIn2D(connected_region);
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

  cv::Vec2f central_axis(e[2],e[3]);
  cv::Vec2f horizontal_axis(e[0],e[1]);

  CheckCentralAxisDirection(center_of_mass,central_axis);
  
  cv::Vec2f normed_central_axis,normed_horizontal_axis;
  cv::normalize(central_axis,normed_central_axis);
  cv::normalize(horizontal_axis,normed_horizontal_axis);
  central_axis = normed_central_axis;
  horizontal_axis = normed_horizontal_axis; 

  if(v[1] < v[0]){
    float tmp = v[1];
    v[1] = v[0];
    v[0] = tmp;
  }

  const float radius = sqrt( (2.0*std::abs(v[0]))/connected_region.size() ); 
  const float length = sqrt( ((12.0*std::abs(v[1])) / connected_region.size())  - 3*radius*radius);

  cv::Vec2f point = cv::Vec2f(center_of_mass) + 0.5*length*central_axis;
  cv::Vec2f top = cv::Vec2f(center_of_mass) + (radius)*horizontal_axis;
  cv::Vec2f bottom = cv::Vec2f(center_of_mass) - (radius)*horizontal_axis;

  cv::Point3f top_unp = cam->UnProjectPoint(cv::Point2i(top));
  cv::Point3f bottom_unp = cam->UnProjectPoint(cv::Point2i(bottom));
  cv::Point3f center_unp = cam->UnProjectPoint(cv::Point2i(center_of_mass));
  cv::Vec3f diff = cv::Vec3f(top_unp) - cv::Vec3f(bottom_unp);
  float abs_diff = sqrt( static_cast<double>( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] ) );

  
  float z = (2*tracked_models_.back().PtrToModel()->Radius())/abs_diff;
  
  cv::Vec3f unp_point = cv::Vec3f(cam->UnProjectPoint(cv::Point2f(point)));
  center_of_mass_3d = cv::Vec3f(cam->UnProjectPoint(cv::Point2f(center_of_mass)));

  cv::line(frame_->GetImageROI(),cv::Point2i(center_of_mass), cv::Point2i(point),cv::Scalar(244,25,30),10);
  cv::circle(frame_->GetImageROI(), cv::Point2i(point),10, cv::Scalar(23,215,30),10);
  //cv::imwrite("test_image.png",frame_->GetImageROI());
  central_axis_3d = unp_point - center_of_mass_3d;
  center_of_mass_3d = center_of_mass_3d * z;
  //std::cerr << cv::Point3f(central_axis_3d) <<"\n";
  //unp_point = unp_point * 50;
  

 
}
void StereoToolTracker::ShiftToTip(const cv::Vec3f &central_axis, cv::Vec3f &center_of_mass) {//, KalmanTracker &tracked_model){

  KalmanTracker t(boost::shared_ptr<Model>(new MISTool(radius_,height_) ));
  const float length_of_central_axis = sqrt( central_axis[0]*central_axis[0] + central_axis[1]*central_axis[1] + central_axis[2]*central_axis[2] );
  boost::shared_ptr<MISTool> mis_tool = boost::static_pointer_cast<MISTool>(t.PtrToModel());

  const cv::Vec3f original_center_of_mass = center_of_mass;
  //cv::Point2i o_com = camera_->rectified_left_eye()->ProjectPointToPixel(cv::Point3f(original_center_of_mass));
  //cv::circle(frame_->GetImageROI(),o_com,20,cv::Scalar(255,0,0),4);
  //std::cerr << "left com = " << cv::Point3f(original_center_of_mass) << "\n";

  float length;
  do{
    
    t.SetPose(center_of_mass,central_axis);
    cv::Vec3f tip_of_instrument = t.CurrentPose().Transform( cv::Vec3f(-this->height_/2 + this->height_*mis_tool->HeightFraction(),0,0) );
    
    //cv::Point2i p = camera_->rectified_left_eye()->ProjectPointToPixel(cv::Point3f(tip_of_instrument));
    //cv::circle(frame_->GetImageROI(),p,10,cv::Scalar(0,244,23),2);

    cv::Vec3f com_to_tip = tip_of_instrument - original_center_of_mass;
    length = sqrt( com_to_tip[0]*com_to_tip[0] + com_to_tip[1]*com_to_tip[1] + com_to_tip[2]*com_to_tip[2] );

    //if () break;

    center_of_mass = center_of_mass - 0.05*central_axis;
    //cv::Point2i n_com = camera_->rectified_left_eye()->ProjectPointToPixel(cv::Point3f(center_of_mass));
    //cv::circle(frame_->GetImageROI(),n_com,15,cv::Scalar(255,125,23),6);
    std::cerr << "length = " <<length << " > " << length_of_central_axis << "\n";
  }while (length > length_of_central_axis);

  //cv::imwrite("point_moves.png",frame_->GetImageROI());
  //std::cerr << "final com = " << cv::Point3f(center_of_mass) << "\n";

  //tracked_model.SetPose(center_of_mass,central_axis);
  //cv::Vec3f tip_of_instrument = tracked_model.CurrentPose().Transform( cv::Vec3f(this->length_/2,0,0) );



}


void StereoToolTracker::Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, KalmanTracker &tracked_model) {

  cv::Vec3f left_center_of_mass,left_central_axis,right_center_of_mass,right_central_axis;
  InitIn2D(region,left_center_of_mass,left_central_axis,camera_->rectified_left_eye());
  
  cv::Mat right_classification_window = StereoFrame()->GetRightClassificationMap();
  std::vector<std::vector<cv::Vec2i> > connected_regions_right_frame;
  FindConnectedRegions(right_classification_window,connected_regions_right_frame);
  /* NOTE - WRITE A CHECK TO FIND THE MATCHING REGIONS - RIGHT NOW WE CAN HACK IT AS THERE SHOULD ONLY BE ONE REGION */
  InitIn2D(connected_regions_right_frame.front(),right_center_of_mass,right_central_axis,camera_->rectified_right_eye());

  cv::Vec3f center_of_mass_3d = left_center_of_mass;//camera_->ReprojectPointTo3D( camera_->rectified_left_eye()->ProjectPointToPixel(cv::Point3f(left_center_of_mass)), camera_->rectified_right_eye()->ProjectPointToPixel(cv::Point3f(right_center_of_mass)) );

  //ShiftToTip(left_central_axis,center_of_mass_3d);
  left_central_axis[2] = -0.5*left_central_axis[0];

  std::cerr << "Center of amass = " << cv::Point3f(center_of_mass_3d) << "\n";
  tracked_model.SetPose(center_of_mass_3d,left_central_axis);

  //sv::Quaternion q(boost::math::quaternion<double>(0.309,0.03218,-0.9324,-0.18433));
  //q = q.Normalize();
  //tracked_model.SetPose(Pose(cv::Vec3f(-18,-18,80),sv::Quaternion(0,cv::Vec3f(1,0,0))));//q));
  //tracked_model.SetPose(Pose(cv::Vec3f(18,-18,90),q));//

  //these values align well for the new_video dataset
  //sv::Quaternion q(boost::math::quaternion<double>(0.309,0.03218,-0.9324,-0.18433));
  //q = q.Normalize();
  //tracked_model.SetPose( Pose(cv::Vec3f(10.349,1.822,43.498),q));

  return;

}

cv::Vec3f StereoToolTracker::FindPrincipalAxisFromMOITensor(const cv::Vec3f center_of_mass_, const cv::Mat &point_cloud) const {

  cv::Matx<float,1,3> principal_axis(0,0,0);
  cv::Matx<float,1,3> center_of_mass(center_of_mass_[0],center_of_mass_[1],center_of_mass_[2]);
  cv::Matx<float,3,3> moi(3,3,CV_32FC1);
  cv::Matx<float,3,3> E = cv::Matx<float,3,3>::eye();
  
  const int rows = point_cloud.rows;
  const int cols = point_cloud.cols;
  const int chans = point_cloud.channels();
  assert(point_cloud.type() == CV_32FC3);

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      
      if( point_cloud.at<cv::Vec3f>(r,c) == cv::Vec3f(0,0,0) || point_cloud.at<cv::Vec3f>(r,c)[2] < 0 ) continue;
      const cv::Matx<float,1,3> point = point_cloud.at<cv::Matx<float,1,3> >(r,c) - center_of_mass;
      moi = moi + ((point*point.t())(0,0)*E) - (point.t()*point);
      
    }
  }

  cv::Mat eigenvectors,eigenvalues;
  cv::eigen(moi,true,eigenvalues,eigenvectors);
  
  int row = 1;
  return cv::Vec3f(eigenvectors.at<float>(row,0),eigenvectors.at<float>(row,1),eigenvectors.at<float>(row,2));

}

cv::Vec3f StereoToolTracker::FindCenterOfMass(const cv::Mat &point_cloud) const {

  const int rows = point_cloud.rows;
  const int cols = point_cloud.cols;
  const int chans = point_cloud.channels();
  assert(point_cloud.type() == CV_32FC3);
  
  cv::Vec3f com(0,0,0);
  size_t num_pts = 0;

  std::vector<cv::Vec3f> pts;

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      const cv::Vec3f &pt = point_cloud.at<cv::Vec3f>(r,c);
      
      //const cv::Point2i p = camera_->rectified_left_eye().ProjectPointToPixel(cv::Point3f(pt));
      
      if( pt != cv::Vec3f(0,0,0) ){
        com += pt;
        num_pts++;
      }
      

    }
  }
  
  com[0] = com[0]/num_pts;
  com[1] = com[1]/num_pts;
  com[2] = com[2]/num_pts;

  return com;

}

void StereoToolTracker::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

  std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();
  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

    cv::Vec2f projected = camera_->rectified_left_eye()->ProjectPoint(point->vertex_);

    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){
      
      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2f projected_neighbour = camera_->rectified_left_eye()->ProjectPoint( neighbour.vertex_ );

      if(canvas.channels() == 3)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),1,CV_AA);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
    }
  }

}


 void StereoToolTracker::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){

   //first set the handle using the superclass method
   Tracker::SetHandleToFrame(image);
   boost::shared_ptr<sv::StereoFrame> stereo_image = boost::dynamic_pointer_cast<sv::StereoFrame>(image);
   
#if defined(SAVEDEBUG_2) 
  cv::imwrite("debug/distorted_left.png",stereo_image->GetLeftImage());
  cv::imwrite("debug/distorted_right.png",stereo_image->GetRightImage());
#endif

  

   //then rectify the camera and remap the images
   /*
   if( !camera_->IsRectified() ) camera_->Rectify(stereo_image->GetLeftImage().size());

   camera_->RemapLeftFrame(stereo_image->GetLeftImage());
   camera_->RemapRightFrame(stereo_image->GetRightImage());
   camera_->RemapLeftFrame(stereo_image->GetLeftClassificationMap());
   camera_->RemapRightFrame(stereo_image->GetRightClassificationMap());
   */
   

   
#if defined(SAVEDEBUG_2) 
   //check this modifies correctly
   cv::imwrite("debug/left.png",stereo_image->GetLeftImage());
   cv::imwrite("debug/right.png",stereo_image->GetRightImage());
   cv::imwrite("debug/classified.png",stereo_image->GetLeftClassificationMap());
#endif
   
 }
 
