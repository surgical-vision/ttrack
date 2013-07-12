#include "../../../headers/track/stt/surgical_tool_tracker.hpp"
#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../deps/image/image/image.hpp"

using namespace ttrk;


SurgicalToolTracker::SurgicalToolTracker(const int radius, const int height):radius_(radius),height_(height){

}

void SurgicalToolTracker::GetContours(const cv::Mat &image, std::vector<std::vector<cv::Point> > &contours) const {

  //threshold the image to 0 & 255 values
  cv::Mat thresholded,thresholded8bit;
  threshold(image,thresholded,127,255,cv::THRESH_BINARY);
  thresholded.convertTo(thresholded8bit,CV_8U);
 
  //find contours around the 255 'blobs' in the image
  findContours(thresholded8bit,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
}

void SurgicalToolTracker::FindSingleRegionFromContour(const std::vector<cv::Point> &contour,std::vector<cv::Vec2i> &connected_region) const {
    
  //we're not intested in the small contours as they correspond to small blobs
  if(contour.size() < 100) return;

  //draw the interior region of the contour on a blank frame
  cv::Point center = GetCenter<cv::Point>(contour);
  cv::Mat mask = cv::Mat::zeros(frame_->rows()+2,frame_->cols()+2,CV_8UC1);
  std::vector<std::vector<cv::Point> >t;
  t.push_back(contour);
  drawContours(mask,t,-1,cv::Scalar(255),CV_FILLED,8);

  //push the interior pixels of the drawn region (faster than polygon intersection?)
  unsigned char *mask_ptr = (unsigned char *)mask.data;
  for(int r=0;r<mask.rows;r++){
    for(int c=0;c<mask.cols;c++){
      if(mask_ptr[r*mask.cols + c] == static_cast<unsigned char>(255))
        connected_region.push_back(cv::Vec2i(c,r));
    }
  }

}

bool SurgicalToolTracker::FindConnectedRegions(const cv::Mat &image,std::vector<std::vector<cv::Vec2i> >&connected_regions){

  //draw contours around the connected regions returning a vector of each connected region
  std::vector<std::vector<cv::Point> >contours;
  GetContours(image,contours);
  
  //iterate over the contours, finding the number of pixels they enclose and store those above a heuristic threshold
  for(size_t i=0;i<contours.size();i++){
   
    std::vector<cv::Vec2i> connected_region;
    FindSingleRegionFromContour(contours[i],connected_region);
    
    if(connected_region.size() > 0.02*image.rows*image.cols) connected_regions.push_back(connected_region);
    
  } 

  //did we find any that were big enough?
  return connected_regions.size() > 0;

}
