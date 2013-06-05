#include "../../../headers/track/stt/stereo_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include <fstream>

using namespace ttrk;

StereoToolTracker::StereoToolTracker(const int radius, const int height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_(calibration_filename){

  localizer_.reset(new StereoPWP3D);
  
}

void StereoToolTracker::CreateDisparityImage(){

  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
  const cv::Mat &left_image = stereo_frame->LeftMat();
  const cv::Mat &right_image = stereo_frame->RightMat();
  cv::Mat out_disparity;

  const int min_disp = 0;
  const int max_disp = 80; //must be exactly divisible by 16
  const int sad_win_size = 7;
  const int smoothness = left_image.channels()*sad_win_size*sad_win_size;
  cv::StereoSGBM sgbm(min_disp,
                      max_disp-min_disp,
                      sad_win_size,
                      8*smoothness,
                      32*smoothness,
                      4, //disp12MaxDiff
                      50, //preFilterCap
                      50, //uniquenessRatio [5-15]
                      4, //speckleWindowSize [50-200]
                      1,//speckleRange
                      true);

  sgbm(left_image,right_image,out_disparity);

  //opencv sgbm multiplies each val by 16 so scale down to floating point array
  //out_disparity.convertTo(*(frame_->PtrToDisparity()),CV_32F,1.0/16);
  *(StereoFrame()->PtrToDisparityMap()) = out_disparity;

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

    KalmanTracker new_tracker;
    new_tracker.model_.reset( new MISTool(radius_,height_) );
    tracked_models_.push_back( new_tracker ); 
    Init3DPoseFromMOITensor(*connected_region, tracked_models_.back());//,corresponding_connected_region);
  
  }

  return false;
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

void StereoToolTracker::Init3DPoseFromMOITensor(const std::vector<cv::Vec2i> &region, KalmanTracker &tracked_model) {

  CreateDisparityImage();
  
  camera_.ReprojectTo3D(*(StereoFrame()->PtrToDisparityMap()),*(StereoFrame()->PtrToPointCloud()),region);
  
  cv::Vec3f center_of_mass = FindCenterOfMass(StereoFrame()->PtrToPointCloud());
  cv::Vec3f center_of_mass_ = FindClusterMode(StereoFrame()->PtrToPointCloud(),StereoFrame()->PtrToClassificationMap());

  //find the central axis of the blob
  cv::Vec3f central_axis = FindPrincipalAxisFromMOITensor(center_of_mass,StereoFrame()->PtrToPointCloud());
  
  cv::Point3f start = (cv::Point3f)(center_of_mass + 10*central_axis);
  cv::Point3f end = (cv::Point3f)(center_of_mass - 10*central_axis);

  std::vector<cv::Point3f> inpoint; //inpoint.push_back( (cv::Point3f)center_of_mass);
  inpoint.push_back(start);
  inpoint.push_back(end);
  /*const int rows = StereoFrame()->PtrToPointCloud()->rows;
  const int cols = StereoFrame()->PtrToPointCloud()->cols;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      inpoint.push_back( (cv::Point3f)StereoFrame()->PtrToPointCloud()->at<cv::Vec3f>(r,c));
    }
  }*/
  cv::Mat r = cv::Mat::eye(3,3,CV_64FC1);
  cv::Mat t = cv::Mat::zeros(1,3,CV_64FC1);
  cv::Mat dist = cv::Mat::zeros(1,4,CV_64FC1);
  cv::Mat proj = camera_.GetP1();
  cv::Point2f outpoint_;
  std::vector<cv::Point2f> outpoint; outpoint.push_back(outpoint_);
  cv::projectPoints(inpoint,
                    r,
                    t,
                    camera_.GetP1(),
                    dist,
                    outpoint);

  //for(auto i = outpoint.begin(); i!=outpoint.end();i++)
  //  cv::circle((StereoFrame()->LeftMat()),*i,1,cv::Scalar(178,44,78),10);
  cv::line((StereoFrame()->LeftMat()),outpoint[0],outpoint[1],cv::Scalar(178,44,78),5);

  cv::imwrite("com.png",StereoFrame()->LeftMat());
  


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
      
      const cv::Matx<float,1,3> point = point_cloud->at<cv::Matx<float,1,3> >(r,c) - center_of_mass;
      moi = moi + ((point*point.t())(0,0)*E) - (point.t()*point);
      
    }
  }

  cv::Mat eigenvectors,eigenvalues;
  cv::eigen(moi,true,eigenvalues,eigenvectors);
  
  if(eigenvectors.type() != CV_32FC1) throw(std::runtime_error("eigenvector not float\n"));
  //return cv::Vec3f(principal_axis(0),principal_axis(1),principal_axis(2));
  int row = 1;
  return cv::Vec3f(eigenvectors.at<float>(row,0),eigenvectors.at<float>(row,1),eigenvectors.at<float>(row,2));

}

cv::Vec3f StereoToolTracker::FindCenterOfMass(const boost::shared_ptr<cv::Mat> point_cloud) const {

  const int rows = point_cloud->rows;
  const int cols = point_cloud->cols;
  const int chans = point_cloud->channels();
  assert(point_cloud->type() == CV_32FC3);
  
  cv::Vec3f com(0,0,0);
  int num_pts = 0;

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      const cv::Vec3f &pt = point_cloud->at<cv::Vec3f>(r,c);
      com += pt;
      num_pts += pt != cv::Vec3f(0,0,0);

    }
  }

  for(int n=0;n<chans;n++) 
    com[n]/= num_pts;

  return com;

}


 void StereoToolTracker::SetHandleToFrame(boost::shared_ptr<sv::Frame> image){

   //first set the handle using the superclass method
   Tracker::SetHandleToFrame(image);
   boost::shared_ptr<sv::StereoFrame> stereo_image = boost::dynamic_pointer_cast<sv::StereoFrame>(image);
   
   //then rectify the camera and remap the images
   if( !camera_.IsRectified() ) camera_.Rectify(stereo_image->LeftMat().size());
   camera_.RemapLeftFrame(stereo_image->LeftMat());
   camera_.RemapRightFrame(stereo_image->RightMat());
   camera_.RemapLeftFrame(stereo_image->ClassificationMap());
 }
 
