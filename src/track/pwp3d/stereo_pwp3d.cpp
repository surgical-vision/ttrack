#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;


Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  const int NUM_STEPS = 1;
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

    cv::Mat front_view = cv::Mat::zeros(sdf_image.size(),CV_8UC1);
    cv::Mat back_view = cv::Mat::zeros(sdf_image.size(),CV_8UC1);

    for(int r=0;r<ROI_left_.rows;r++){
      for(int c=0;c<ROI_left_.cols;c++){

        const double region_agreement = GetRegionAgreement(r,c,sdf_image.at<float>(r,c),norm_foreground, norm_background);

        const cv::Mat pose_derivatives = GetPoseDerivatives(r,c,dSDFdx.at<float>(r,c),dSDFdy.at<float>(r,c),sdf_image.at<float>(r,c),current_model);

        cv::Vec2f projected_front = camera_->rectified_left_eye().ProjectPoint(cv::Point3f(pose_derivatives.at<double>(0,0),pose_derivatives.at<double>(1,0),pose_derivatives.at<double>(2,0)));
        cv::Vec2f projected_back = camera_->rectified_left_eye().ProjectPoint(cv::Point3f(pose_derivatives.at<double>(3,0),pose_derivatives.at<double>(4,0),pose_derivatives.at<double>(5,0)));

        if(cv::Point3f(pose_derivatives.at<double>(0,0),pose_derivatives.at<double>(1,0),pose_derivatives.at<double>(2,0)) != cv::Point3f(0,0,0))
          front_view.at<unsigned char>(projected_front[1],projected_front[0]) = 255;      
        if(cv::Point3f(pose_derivatives.at<double>(3,0),pose_derivatives.at<double>(4,0),pose_derivatives.at<double>(5,0)) != cv::Point3f(0,0,0))
          back_view.at<unsigned char>(projected_back[1],projected_back[0]) = 255;

        const cv::Mat regularized_depth = GetRegularizedDepth(r,c);

        //jacobian = jacobian + (region_agreement*pose_derivatives) + regularized_depth;

      }
    }

    cv::imwrite("front.png",front_view);
    cv::imwrite("back.png",back_view);

    ScaleJacobian(jacobian);
    ApplyGradientDescentStep(jacobian);

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

      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);

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
  cv::Mat ret(6,1,CV_64FC1);
  //for(int dof=0;dof<6;dof++){
  for(int c=0;c<3;c++){
    ret.at<double>(c,0) = front_intersection[c];
    ret.at<double>(3+c,0) = back_intersection[c];
  }

 //}


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
