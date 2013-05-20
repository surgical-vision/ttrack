#include "../../../headers/track/stt/stereo_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"

using namespace ttrk;

StereoToolTracker::StereoToolTracker(const int radius, const int height, const std::string &calibration_filename):SurgicalToolTracker(radius,height),camera_(calibration_filename){

  localizer_.reset(new StereoPWP3D);
  
}

bool StereoToolTracker::Init() {

  boost::shared_ptr<tcv::StereoImage<unsigned char,3> > stereo_frame_ = boost::dynamic_pointer_cast<tcv::StereoImage<unsigned char,3> >(frame_);

  std::vector<std::vector<cv::Vec2i> >connected_regions;
  if(!FindConnectedRegions(*stereo_frame_->LeftMat(),connected_regions)) return false;

  for(auto connected_region = connected_regions.cbegin(); connected_region != connected_regions.end(); connected_region++){

    std::vector<cv::Vec2i> corresponding_connected_region;
    const cv::Vec2i center_of_region = GetCenter<cv::Vec2i>(*connected_region);

    FindConnectedRegionsFromSeed(*stereo_frame_->RightMat(), center_of_region, corresponding_connected_region);
    
    KalmanTracker new_tracker;
    new_tracker.model_.reset( new MISTool(radius_,height_) );

    tracked_models_.push_back( new_tracker ); 

    Init3DPoseFromDualMOITensor(*connected_region,corresponding_connected_region);

  }

  return false;

}

void StereoToolTracker::Init3DPoseFromDualMOITensor(const std::vector<cv::Vec2i> &region_left, const std::vector<cv::Vec2i> &region_right) {

  

}

void StereoToolTracker::FindConnectedRegionsFromSeed(const cv::Mat &image, const cv::Vec2i &seed, std::vector<cv::Vec2i> &connected_region){
  
  std::vector<std::vector<cv::Point> >contours;
  cv::Mat thresholded;
  threshold(*frame_->Mat(),thresholded,127,255,cv::THRESH_BINARY);
  findContours(thresholded,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  for(size_t i=0;i<contours.size();i++){
    std::vector<cv::Point> &contour = contours[i];
    if(contour.size() < 100) continue;
    cv::Point center = GetCenter<cv::Point>(contour);
    cv::Mat mask = cv::Mat::zeros(frame_->rows()+2,frame_->cols()+2,CV_8UC1);
    std::vector<std::vector<cv::Point> >t;
    t.push_back(contour);
    drawContours(mask,t,-1,cv::Scalar(255),CV_FILLED,8);

    if( mask.at<unsigned char>(seed[1],seed[0]) == 255 ){
      unsigned char *mask_ptr = (unsigned char *)mask.data;
      for(int r=0;r<mask.rows;r++){
        for(int c=0;c<mask.cols;c++){
          if(mask_ptr[r*mask.cols + c] == static_cast<unsigned char>(255))
            connected_region.push_back(cv::Vec2i(c,r));
        }
      }
    }

  } 



}
