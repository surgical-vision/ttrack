#include "../../../include/track/stt/surgical_tool_tracker.hpp"
#include "../../../include/track/pwp3d/pwp3d.hpp"
#include "../../../deps/image/image/image.hpp"
#include <boost/filesystem.hpp>

using namespace ttrk;


SurgicalToolTracker::SurgicalToolTracker(const std::string &model_parameter_file){

  if(!boost::filesystem::exists(boost::filesystem::path(model_parameter_file)))
    throw(std::runtime_error("Error, unable to read model file: " + model_parameter_file + "\n"));

  model_parameter_file_ = model_parameter_file;

}

void SurgicalToolTracker::GetContours(const cv::Mat &image, std::vector<std::vector<cv::Point> > &contours) const {

  //threshold the image to 0 & 255 values
  cv::Mat thresholded,thresholded8bit;
  threshold(image,thresholded,50,255,cv::THRESH_BINARY);
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

inline bool PointInImage(const cv::Point2i &image_coords, const cv::Size &image_size){
  return cv::Rect(cv::Point(0,0),image_size).contains(image_coords);
}

void SurgicalToolTracker::ShiftCenter(cv::Vec2d &center_of_mass,const cv::Vec2d &central_axis, double length) const {
  
  const cv::Mat &ci = frame_->GetClassificationMapROI();
  cv::Vec2d end = center_of_mass + 0.5*length*central_axis;

  //if end off image
  //iterate the length back until it is on the image
  bool estimate_short = PointInImage(cv::Point((int)end[0],(int)end[1]),ci.size()) && ci.at<unsigned char>((int)end[1],(int)end[0]) > 127;
  double update_direction = -0.05;
  if(estimate_short)
    update_direction *= -1;
  
  bool new_estimate;
  int step = -1;
  double new_length = length;
  do{

    new_length = new_length*(1+update_direction);
    end = center_of_mass + (0.5*new_length)*central_axis;
    new_estimate = PointInImage(cv::Point((int)end[0],(int)end[1]),ci.size()) && ci.at<unsigned char>((int)end[1],(int)end[0]) > 127;
    step++;

  }while(estimate_short == new_estimate && step < 15);

  center_of_mass = end - (0.5*length)*central_axis;

}

void SurgicalToolTracker::CheckCentralAxisDirection(const cv::Vec2d &center_of_mass, cv::Vec2d &central_axis) const {

  
  const cv::Vec2i center_of_image((int)((float)frame_->GetImageROI().cols/2),(int)((float)frame_->GetImageROI().rows/2));
  const cv::Vec2d center_of_mass_to_center_of_image = cv::Vec2d(center_of_image) - cv::Vec2d(center_of_mass);

  const double cos_angle =  center_of_mass_to_center_of_image.dot(central_axis) ; //we don't care about scale factor so don't bother to normalize
  const double cos_angle_reversed =  center_of_mass_to_center_of_image.dot(-central_axis) ;

  if( cos_angle > cos_angle_reversed ) return; //we seek the smallest angle (largest cos(angle))
  else central_axis *= -1;

}

bool SurgicalToolTracker::FindConnectedRegions(const cv::Mat &image,std::vector<std::vector<cv::Vec2i> >&connected_regions){

  //draw contours around the connected regions returning a vector of each connected region
  std::vector<std::vector<cv::Point> >contours;
     
  GetContours(image,contours);
  
  //iterate over the contours, finding the number of pixels they enclose and store those above a heuristic threshold
  for(size_t i=0;i<contours.size();i++){
   
    std::vector<cv::Vec2i> connected_region;
    FindSingleRegionFromContour(contours[i],connected_region);
    
    if(connected_region.size() > 0.028*image.rows*image.cols) connected_regions.push_back(connected_region);
      
  } 
  
  //did we find any that were big enough?
  return connected_regions.size() > 0;

}
