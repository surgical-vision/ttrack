#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
using namespace ttrk;


const cv::Mat MonoPWP3D::ProjectShapeToSDF(KalmanTracker &current_model) {

  std::vector<SimplePoint<> > points = current_model.ModelPointsAtCurrentPose();
  std::vector<cv::Vec2i > projected_points( points.size() );

  for(size_t i=0;i<points.size();i++){

    projected_points.push_back( camera_->ProjectPointToPixel(points[i].vertex_) );

  }
  
  std::vector<cv::Vec2i> convex_hull;
  cv::convexHull(projected_points,convex_hull);
  cv::Mat convex_hull_(convex_hull);

  FindROI(convex_hull); //POTENTIAL OPTIMISATION: find a ROI around the target object to not run tracking over whole image. Not yet implemented.

  //alternatively use cv::distanceTransform?
  cv::Mat sdf_image(ROI.size(),CV_32FC1);

  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){

      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);

    }
  }

  return sdf_image;

}

void MonoPWP3D::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, KalmanTracker &current_model){

  cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
  
  current_model.PtrToModel()->GetIntersection(ray, front_intersection, back_intersection,current_model.CurrentPose());

}

cv::Mat MonoPWP3D::GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf, KalmanTracker &current_model){

  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);

  for(int dof=0;dof<6;dof++){


    int x = 0;

  }


  return cv::Mat();
}

Pose MonoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  const int NUM_STEPS = 10;
  double energy = std::numeric_limits<double>::max(); //to minimise

  for(int step=0; step < NUM_STEPS; step++){
     
    cv::Mat sdf_image = ProjectShapeToSDF(current_model);

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

        const cv::Mat pose_derivatives = GetPoseDerivatives(r,c,dSDFdx.at<float>(r,c),dSDFdy.at<float>(r,c),sdf_image.at<float>(r,c),current_model);

        jacobian = jacobian + (region_agreement*pose_derivatives);

      }
    }

    ScaleJacobian(jacobian);
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose());

  }

  //return something like current_model.CurrentPose() + delta*Jacobian
  return current_model.CurrentPose();

}

double MonoPWP3D::GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const{

  const double pixel_probability = ROI.at<float>(r,c)/255.0;
  const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability/norm;
  const double background_probability = (1-pixel_probability)/norm;
  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf)*foreground_probability + (1.0-Heaviside(sdf))*background_probability);
  return region_agreement;

}

void MonoPWP3D::ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const {
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

void MonoPWP3D::FindROI(const std::vector<cv::Vec2i> &convex_hull) {

  ROI = frame_->Mat(); //UNTIL I DO THIS FUNCTION

  for(int r=0;r<frame_->rows();r++){
    for(int c=0;c<frame_->cols();c++){



    }
  }
  
}
