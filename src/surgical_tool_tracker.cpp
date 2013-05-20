#include "../headers/surgical_tool_tracker.hpp"
#include "../headers/pwp3d.hpp"
#include <image/image.hpp>

using namespace ttrk;


SurgicalToolTracker::SurgicalToolTracker(const int radius, const int height):radius_(radius),height_(height){

}

cv::Point SurgicalToolTracker::GetCenter(std::vector<cv::Point> &contour) const {
  cv::Point center(0,0);
  for(size_t i=0;i<contour.size();i++){
    center = center + contour[i];
  }
  center.x = center.x/(int)contour.size();
  center.y = center.y/(int)contour.size();
  return center;
}


bool SurgicalToolTracker::FindConnectedRegions(std::vector<std::vector<cv::Vec2i> >&connected_regions){

  std::vector<std::vector<cv::Point> >contours;
  cv::Mat thresholded;
  threshold(*frame_->Mat(),thresholded,127,255,cv::THRESH_BINARY);
  findContours(thresholded,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  for(size_t i=0;i<contours.size();i++){
    std::vector<cv::Point> &contour = contours[i];
    if(contour.size() < 100) continue;
    cv::Point center = GetCenter(contour);
    cv::Mat mask = cv::Mat::zeros(frame_->rows()+2,frame_->cols()+2,CV_8UC1);
    std::vector<std::vector<cv::Point> >t;
    t.push_back(contour);
    drawContours(mask,t,-1,cv::Scalar(255),CV_FILLED,8);

    std::vector<cv::Vec2i> connected_region;
    unsigned char *mask_ptr = (unsigned char *)mask.data;
    for(int r=0;r<mask.rows;r++){
      for(int c=0;c<mask.cols;c++){
        if(mask_ptr[r*mask.cols + c] == static_cast<unsigned char>(255))
          connected_region.push_back(cv::Vec2i(c,r));
      }
    }
    if(connected_region.size() > 2500) connected_regions.push_back(connected_region);
  } 

  return connected_regions.size() > 0;

}

void SurgicalToolTracker::Init2DPoseFromMOITensor(const std::vector<cv::Vec2i> &connected_region){

  const cv::Vec2i center_of_mass = FindCenterOfMass(connected_region);

}

const cv::Vec2i SurgicalToolTracker::FindCenterOfMass(const std::vector<cv::Vec2i> &connected_region) const {

  cv::Vec2i ret(0,0);
  return ret;

}



