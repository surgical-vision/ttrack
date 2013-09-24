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

StereoToolTracker::StereoToolTracker(const float radius, const float height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_( new StereoCamera(calibration_filename)){

  localizer_.reset(new StereoPWP3D);
  boost::shared_ptr<StereoPWP3D> stereo_pwp3d = boost::dynamic_pointer_cast<StereoPWP3D>(localizer_);
  stereo_pwp3d->GetStereoCamera() = camera_; //MOVE THESE TO CONSTRUCTOR
  stereo_pwp3d->Camera() = stereo_pwp3d->GetStereoCamera()->rectified_left_eye();

}

void StereoToolTracker::CreateDisparityImage(){

  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
  cv::Mat &left_image = stereo_frame->LeftMat();
  cv::Mat &right_image = stereo_frame->RightMat();
  
  cv::Mat out_disparity(left_image.rows,left_image.cols,CV_8UC3);

#ifdef QUASI
  QuasiDenseStereo qds;

  qds.initialise( cvSize( left_image.cols, left_image.rows ) );
  qds.Param.BorderX = 15;				// borders around the image
  qds.Param.BorderY = 15;
  qds.Param.N = 5;						// neighbours
  qds.Param.Ct = 0.3;					// corre threshold for seeds
  qds.Param.Dg = 0.5;					// disparity gradient
  qds.Param.WinSizeX = 6;				// corr window size
  qds.Param.WinSizeY = 6;	
  qds.Param.Tt = 100;

  IplImage left = (IplImage)left_image;
  IplImage right = (IplImage)right_image;
  qds.process(&left,&right);
  IplImage disparity = (IplImage)out_disparity;
  qds.getDisparityImage(&disparity);
  
#endif
  
#ifdef STEREO_SGBM
  const int min_disp = 0;
  const int max_disp = 48; //must be exactly divisible by 16
  const int sad_win_size = 12;
  const int smoothness = left_image.channels()*sad_win_size*sad_win_size;
  cv::StereoSGBM sgbm(min_disp,
                      max_disp-min_disp,
                      sad_win_size,
                      8*smoothness,
                      32*smoothness,
                      4,
                      50,
                      10,
                      50,
                      1,
                      true);
  /*,
  //                    4, //disp12MaxDiff
  //                    50, //preFilterCap
  //                    10, //uniquenessRatio [5-15] 50
  //                    50, //speckleWindowSize [50-200] 4
  //                    1,//speckleRange
  //                    true);*/
  
  sgbm(left_image,right_image,out_disparity);
  
#endif

  cv::Mat float_disp(out_disparity.size(),CV_32FC1);
  for(int r=0;r<out_disparity.rows;r++){
    for(int c=0;c<out_disparity.cols;c++){

#ifdef QUASI
      float_disp.at<float>(r,c) = ((float)out_disparity.at<cv::Vec3b>(r,c)[0]);
      if(float_disp.at<float>(r,c) <= 0 || (float)out_disparity.at<cv::Vec3b>(r,c)[0] == 200){
        float_disp.at<float>(r,c) = 0;
      }
#elif defined STEREO_SGBM
      float_disp.at<float>(r,c) = static_cast<float>(out_disparity.at<int16_t>(r,c));///16.0f;
      if(float_disp.at<float>(r,c) <= 0){
        float_disp.at<float>(r,c) = 0;
      }
#endif
    }
  }

  //opencv sgbm multiplies each val by 16 so scale down to floating point array
  *(StereoFrame()->PtrToDisparityMap()) = float_disp;
  //*(StereoFrame()->PtrToDisparityMap()) = out_disparity;
#if defined(SAVEDEBUG)
  cv::imwrite("debug/disparity.png", float_disp);
#endif
}


bool StereoToolTracker::Init() {

  boost::shared_ptr<sv::StereoFrame> stereo_frame_ = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //find the connected regions in the image
  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(*stereo_frame_->PtrToClassificationMap(),connected_regions)) {
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

cv::Vec3f StereoToolTracker::FindClusterMode(const boost::shared_ptr<cv::Mat> point_cloud, const boost::shared_ptr<cv::Mat> classification_map) const {

  cv::Vec3f cluster_mode(0,0,0);

  //mean shift used to find cluster mode
  //weighted average of all points in region
  //start with initial estimate
  //iterate over all points and score by distance from current start point
  //if most of the points are distributed unevenly around the start point
  //the new mean will be estiamted 

  //
  //1) kernel density estimation to find a di
  //sum over 3d region for each voxel estimate a density as --
  //  sum over the image pixels for each point (r,c) ---
  //    create weight w_{i} according to the class probability for (r,c)
  //    compute L1 norm of vector between backprojection of (r,c) and 3d point (x,y,z)
  //    nroamlize by bandwidth 

  return cluster_mode;
  
}

void StereoToolTracker::ProcessFrame(){

  CreateDisparityImage();
  camera_->ReprojectTo3D(*(StereoFrame()->PtrToDisparityMap()),*(StereoFrame()->PtrToPointCloud()),std::vector<cv::Vec2i>());

}

void StereoToolTracker::Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, KalmanTracker &tracked_model) {

  //create the point cloud used to initialize the pose
  CreateDisparityImage();
  camera_->ReprojectTo3D(*(StereoFrame()->PtrToDisparityMap()),*(StereoFrame()->PtrToPointCloud()),region);
  
  //find the center of mass of the point cloud and shift it to the center of the shape rather than lie on the surface
  cv::Vec3f center_of_mass = FindCenterOfMass(StereoFrame()->PtrToPointCloud());
  center_of_mass *= ((cv::norm(center_of_mass) + radius_)/cv::norm(center_of_mass));
  //center_of_mass *= 1.5;
  center_of_mass = cv::Vec3f(9.7,-0.6,48);
  std::cerr << "center of mass = " << cv::Point3f(center_of_mass) << std::endl;
  
  //find the central axis of the point cloud
  //center_of_mass += cv::Vec3f(-3.1,-2.0,4.1);
  cv::Vec3f central_axis = FindPrincipalAxisFromMOITensor(center_of_mass,StereoFrame()->PtrToPointCloud());
  
  central_axis = cv::Vec3f(-1,-0.18,1.05);//-1.4,0.2,-0.2);
  //central_axis += cv::Vec3f(-0.3,-0.1,-0.2);
  cv::Vec3f t_central_axis = central_axis;
  cv::normalize(t_central_axis,central_axis);

  //central_axis = -central_axis;
  std::cerr << "central axis = " << cv::Point3f(central_axis) << std::endl;

  //central_axis = cv::normalize(central_axis);

  //use these two parameters to set the initial pose of the object
  tracked_model.SetPose(center_of_mass,central_axis);
  
}

cv::Vec3f StereoToolTracker::FindPrincipalAxisFromMOITensor(const cv::Vec3f center_of_mass_, const boost::shared_ptr<cv::Mat> point_cloud) const {

  cv::Matx<float,1,3> principal_axis(0,0,0);
  cv::Matx<float,1,3> center_of_mass(center_of_mass_[0],center_of_mass_[1],center_of_mass_[2]);
  cv::Matx<float,3,3> moi(3,3,CV_32FC1);
  cv::Matx<float,3,3> E = cv::Matx<float,3,3>::eye();
  
  const int rows = point_cloud->rows;
  const int cols = point_cloud->cols;
  const int chans = point_cloud->channels();
  assert(point_cloud->type() == CV_32FC3);

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      
      if( point_cloud->at<cv::Vec3f>(r,c) == cv::Vec3f(0,0,0) || point_cloud->at<cv::Vec3f>(r,c)[2] < 0 ) continue;
      const cv::Matx<float,1,3> point = point_cloud->at<cv::Matx<float,1,3> >(r,c) - center_of_mass;
      moi = moi + ((point*point.t())(0,0)*E) - (point.t()*point);
      
    }
  }

  cv::Mat eigenvectors,eigenvalues;
  cv::eigen(moi,true,eigenvalues,eigenvectors);
  
  int row = 1;
  return cv::Vec3f(eigenvectors.at<float>(row,0),eigenvectors.at<float>(row,1),eigenvectors.at<float>(row,2));

}

cv::Vec3f StereoToolTracker::FindCenterOfMass(const boost::shared_ptr<cv::Mat> point_cloud) const {

  const int rows = point_cloud->rows;
  const int cols = point_cloud->cols;
  const int chans = point_cloud->channels();
  assert(point_cloud->type() == CV_32FC3);
  
  cv::Vec3f com(0,0,0);
  size_t num_pts = 0;

  std::vector<cv::Vec3f> pts;

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      const cv::Vec3f &pt = point_cloud->at<cv::Vec3f>(r,c);
      
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
   
#if defined(SAVEDEBUG) 
  cv::imwrite("debug/dist_left.png",stereo_image->LeftMat());
  cv::imwrite("debug/dist_right.png",stereo_image->RightMat());
#endif

   //then rectify the camera and remap the images
   if( !camera_->IsRectified() ) camera_->Rectify(stereo_image->LeftMat().size());
   camera_->RemapLeftFrame(stereo_image->LeftMat());
   camera_->RemapRightFrame(stereo_image->RightMat());
   camera_->RemapLeftFrame(stereo_image->ClassificationMap());
   
#if defined(SAVEDEBUG) 
  cv::imwrite("debug/left.png",stereo_image->LeftMat());
  cv::imwrite("debug/right.png",stereo_image->RightMat());
  cv::imwrite("debug/classified.png",stereo_image->ClassificationMap());
#endif

   //save the roi around the good part of the image to speed up localization
   stereo_image->rectified_region_ = camera_->ROILeft();
     
 }
 
