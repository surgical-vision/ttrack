#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;


Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  const int NUM_STEPS = 2;
  double energy = std::numeric_limits<double>::max(); //to minimise

  for(int step=0; step < NUM_STEPS; step++){
#ifdef DEBUG
    boost::progress_timer t; //timer prints time when it goes out of scope
#endif
    cv::Mat sdf_image = ProjectShapeToSDF(current_model);

    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);

    //compute the derivates of the sdf images
    cv::Mat dSDFdx, dSDFdy;
    cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
    cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);

    cv::Mat front_view = cv::Mat::zeros(sdf_image.size(),CV_8UC1);
    cv::Mat back_view = cv::Mat::zeros(sdf_image.size(),CV_8UC1);

    for(int r=0;r<ROI_left_.rows;r++){
      for(int c=0;c<ROI_left_.cols;c++){

        const double region_agreement = GetRegionAgreement(r,c,sdf_image.at<float>(r,c),norm_foreground, norm_background);

        const cv::Mat pose_derivatives = DeltaFunction(sdf_image.at<float>(r,c))*GetPoseDerivatives(r,c,dSDFdx.at<float>(r,c),dSDFdy.at<float>(r,c),sdf_image.at<float>(r,c),current_model);
        //(x,y,z,w,r1,r2,r3)
        const cv::Mat regularized_depth = GetRegularizedDepth(r,c);

        for(int i=0;i<pose_derivatives.rows;i++)
          jacobian.at<double>(i,0) += region_agreement*pose_derivatives.at<double>(i,0) + regularized_depth.at<double>(i,0);
        
      }
    }

    ScaleJacobian(jacobian);
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose());

  }

  //return something like current_model.CurrentPose() + delta*Jacobian
  return current_model.CurrentPose();

}

cv::Mat StereoPWP3D::GetRegularizedDepth(const int r, const int c) const {
  cv::Mat x = cv::Mat::zeros(7,1,CV_64FC1);
  return x;
}

const cv::Mat StereoPWP3D::ProjectShapeToSDF(KalmanTracker &current_model) {

  std::vector<SimplePoint<> > points = current_model.ModelPointsAtCurrentPose();
  std::vector<cv::Vec2i > projected_points;//( points.size() );

  for(size_t i=0;i<points.size();i++){
    cv::Point2f pt = camera_->rectified_left_eye().ProjectPoint(points[i].vertex_);
    projected_points.push_back( cv::Vec2i( pt.x,pt.y) );
  }

  std::vector<cv::Vec2i> convex_hull;
  cv::convexHull(projected_points,convex_hull);
  cv::Mat convex_hull_(convex_hull);

  FindROI(convex_hull); //POTENTIAL OPTIMISATION: find a ROI around the target object to not run tracking over whole image. Not yet implemented.

  //alternatively use cv::distanceTransform?
  cv::Mat sdf_image(ROI_left_.size(),CV_32FC1);

  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){

      sdf_image.at<float>(r,c) = HeavisideFunction((float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true));

    }
  }
  
  return sdf_image;

}

void StereoPWP3D::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, KalmanTracker &current_model){

  cv::Vec3f ray = camera_->rectified_left_eye().UnProjectPoint( cv::Point2i(c,r) ); 
  
  current_model.PtrToModel()->GetIntersection(ray, front_intersection, back_intersection,current_model.CurrentPose());

}

cv::Mat StereoPWP3D::GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf, KalmanTracker &current_model){

  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  if(front_intersection == cv::Vec3f(0,0,0)) return cv::Mat::zeros(6,2,CV_64FC1);
  cv::Mat ret(6,2,CV_64FC1);
  for(int dof=0;dof<6;dof++){

    cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);
      
    const double dXdL = camera_->rectified_left_eye().Fx() * ((1.0/(front_intersection[2]*front_intersection[2]))*(front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2]));
    const double dYdL = camera_->rectified_left_eye().Fy() * ((1.0/(front_intersection[2]*front_intersection[2]))*(front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2]));
    ret.at<double>(dof,0) = dSDFdx * dXdL;
    ret.at<double>(dof,1) = dSDFdy * dYdL;
  
  }


  return ret;
}

double StereoPWP3D::GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const{
  
  const double pixel_probability = (double)frame_->ClassificationMap().at<unsigned char>(r,c)/255.0;
  const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability/norm;
  const double background_probability = (1-pixel_probability)/norm;
  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf)*foreground_probability + (1.0-Heaviside(sdf))*background_probability);
  return region_agreement;

}

void StereoPWP3D::ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const {
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



void StereoPWP3D::FindROI(const std::vector<cv::Vec2i> &convex_hull) {

  ROI_left_ = frame_->Mat(); //UNTIL I DO THIS FUNCTION

  for(int r=0;r<frame_->rows();r++){
    for(int c=0;c<frame_->cols();c++){



    }
  }
  
}
