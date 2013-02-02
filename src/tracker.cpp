#include "../headers/tracker.hpp"

using namespace ttrk;

Tracker::Tracker(){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);

  if(!tracking_ && !Init() && InitKalmanFilter()) //do any custom initalisation in the virtual Init function
    return;
  
  
  InitKalmanFilter();

  circle(*frame_,cv::Point(400,100),20,cv::Scalar(200,182,233),-1);
  
}  

bool Tracker::InitKalmanFilter(){

  return true;


}

const cv::Mat Tracker::ProjectShapeToSDF(const int image_width, const int image_height) const {

  std::vector<cv::Vec3f> points = (*current_model_)->Points();
  std::vector<cv::Vec2i> projected_points( points.size() );

  for(size_t i=0;i<points.size();i++){

    projected_points.push_back( camera_->left_eye().ProjectPointToPixel(points[i]) );

  }
  
  std::vector<cv::Vec2i> convex_hull;
  cv::convexHull(projected_points,convex_hull);
  cv::Mat convex_hull_(convex_hull);

  //alternatively use cv::distanceTransform?
  cv::Mat sdf_image(image_height,image_width,CV_32FC1);

  for(int r=0;r<image_height;r++){
    for(int c=0;c<image_width;c++){

      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);

    }
  }

  return sdf_image;

}

void Tracker::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  frame_.reset();
  frame_ = image;

}

/*bool Tracker::Init(){
  //pose init
  bool found = FindConnectedRegions();

  InitPoseFromMOITensor();

}*/

boost::shared_ptr<cv::Mat> Tracker::GetPtrToFinishedFrame(){
  return frame_;
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}
  
