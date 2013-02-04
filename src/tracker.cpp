#include "../headers/tracker.hpp"
#include "../headers/helpers.hpp"

using namespace ttrk;

Tracker::Tracker(const std::string &calibration_filename):camera_(calibration_filename){}

Tracker::~Tracker(){}

void Tracker::operator()(boost::shared_ptr<cv::Mat> image){
  
  SetHandleToFrame(image);

  if(!tracking_){
    if(!Init() || !InitKalmanFilter()) //do any custom initalisation in the virtual Init function
      return;
    tracking_ = true;
  }
  
  //track each model that we know about
  for(current_model_ = tracked_models_.begin(); current_model_ != tracked_models_.end(); current_model_++ ){
    
    cv::Mat pose_estimate = TrackTargetInFrame();
  
    UpdatePose(pose_estimate);
  
  }
}  


cv::Mat Tracker::TrackTargetInFrame(){

  const int NUM_STEPS = 10;
  double energy = std::numeric_limits<double>::max(); //to minimise

  for(int step=0; step < NUM_STEPS; step++){
     
    cv::Mat sdf_image = ProjectShapeToSDF();

    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);

    //compute the derivates of the sdf images
    cv::Mat dSDFdx, dSDFdy;
    cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
    cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

    cv::Mat jacobian = cv::Mat::zeros(1,6,CV_64FC1);

    for(int r=0;r<ROI.rows;r++){
      for(int c=0;c<ROI.cols;c++){

        const double region_agreement = GetRegionAgreement(r,c,sdf_image.at<float>(r,c),norm_foreground, norm_background);

        const cv::Mat pose_derivatives = GetPoseDerivatives(r,c,dSDFdx.at<float>(r,c),dSDFdy.at<float>(r,c),sdf_image.at<float>(r,c));

        jacobian = jacobian + (region_agreement*pose_derivatives);

      }
    }

    ScaleJacobian(jacobian);
    ApplyGradientDescentStep(jacobian);

  }

  return GetPoseUpdateEstimate();

}

void Tracker::ApplyGradientDescentStep(const cv::Mat &jacobian){

  //update the temporary update pose



}

cv::Mat Tracker::GetPoseUpdateEstimate() {
  
  cv::Mat original_pose = current_model_->model_->Pose();
  cv::Mat new_pose = current_model_->temporary_update_pose;
  cv::Mat difference;

  //GET TEH DIFFERENCE

  return difference;

}

double Tracker::GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const{

  const double pixel_probability = ROI.at<float>(r,c)/255.0;
  const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability/norm;
  const double background_probability = (1-pixel_probability)/norm;
  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf)*foreground_probability + (1.0-Heaviside(sdf))*background_probability);
  return region_agreement;

}


void Tracker::ScaleJacobian(cv::Mat &jacobian) const{

  assert(jacobian.type() == CV_64FC1); 
  const double rotation_scale = 0.00428868;
  const double translation_scale = 0.0023718;

  for(int rot=0;rot<3;rot++)
    jacobian.at<double>(0,rot) *= (-1*rotation_scale);

  for(int tran=3;tran<6;tran++)
    jacobian.at<double>(0,tran) *= (-1*translation_scale);

}

void Tracker::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection){

  cv::Vec3f ray = camera_.left_eye().UnProjectPoint( cv::Point2i(c,r) );
  
  current_model_->model_->GetIntersection(ray, front_intersection, back_intersection);

}

cv::Mat Tracker::GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf){

  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  GetTargetIntersections(r,c,front_intersection,back_intersection);

  for(int dof=0;dof<6;dof++){




  }


  return cv::Mat();
}
   

void Tracker::ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const {
  norm_foreground = norm_background = 0.0;
  const int width = sdf_image.cols, height = sdf_image.rows;

  for(int r=0;r<height;r++){
    for(int c=0;c<width;c++){

      double sdf_pixel = sdf_image.at<float>(r,c);
      norm_foreground += Heaviside(sdf_pixel);
      norm_background += 1.0 - Heaviside(sdf_pixel);

    }
  }

}

void Tracker::UpdatePose(const cv::Mat &pose_estimate){

  // get the prediction from the kalman filter

  // combine with the measurement

  // update the pose

}


bool Tracker::InitKalmanFilter(){

  for(int i = 0; i<tracked_models_.size(); i++){
  
    KalmanTracker &track = tracked_models_[i];
    cv::Mat initial_pose = track.model_->Pose();
    // set up kalman filter specific stuff
    
    track.filter_.init( 6 , 6 );

    
  }
    
  return true;

}


void Tracker::FindROI(const std::vector<cv::Vec2i> &convex_hull) {

  ROI = *frame_; //UNTIL I DO THIS FUNCTION

  for(int r=0;r<frame_->rows;r++){
    for(int c=0;c<frame_->cols;c++){



    }
  }
  
}


const cv::Mat Tracker::ProjectShapeToSDF() {

  std::vector<SimplePoint<> > points = (*current_model_).model_->Points();
  std::vector<cv::Vec2i > projected_points( points.size() );

  for(size_t i=0;i<points.size();i++){

    projected_points.push_back( camera_.left_eye().ProjectPointToPixel(points[i].vertex_) );

  }
  
  std::vector<cv::Vec2i> convex_hull;
  cv::convexHull(projected_points,convex_hull);
  cv::Mat convex_hull_(convex_hull);

  FindROI(convex_hull);

  //alternatively use cv::distanceTransform?
  cv::Mat sdf_image(ROI.size(),CV_32FC1);

  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){

      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);

    }
  }

  return sdf_image;

}

void Tracker::SetHandleToFrame(boost::shared_ptr<cv::Mat> image){

  frame_.reset();
  frame_ = image;

}


boost::shared_ptr<cv::Mat> Tracker::GetPtrToFinishedFrame(){
  return frame_;
}

void Tracker::Tracking(const bool toggle){
  tracking_ = toggle;
}
  
