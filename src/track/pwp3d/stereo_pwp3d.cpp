#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;


/*** REMOVE THIS ***/


void StereoPWP3D::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

  std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();
  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

    cv::Vec2f projected = camera_->rectified_left_eye().ProjectPoint(point->vertex_);

    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){
      
      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2f projected_neighbour = camera_->rectified_left_eye().ProjectPoint( neighbour.vertex_ );

      if(canvas.channels() == 3)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),3,8);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,3,8);
    }
  }

}



Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  const int NUM_STEPS = 1;

  /*** TO DELETE ***/
  cv::Mat canvas = frame_->Mat().clone();
  DrawModelOnFrame(current_model,canvas);
  cv::imwrite("step_init.png",canvas);

  for(int step=0; step < NUM_STEPS; step++){

#ifdef DEBUG
    boost::progress_timer t; //timer prints time when it goes out of scope
#endif

    cv::Mat sdf_image = ProjectShapeToSDF(current_model);

    //compute the normalization values n_f and n_b
    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);

    //compute the derivates of the sdf images
    cv::Mat dSDFdx, dSDFdy;
    cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
    cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);

    for(int r=0;r<ROI_left_.rows;r++){
      for(int c=0;c<ROI_left_.cols;c++){

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c), norm_foreground, norm_background);

        //dH / dL
        const cv::Mat pose_derivatives = DeltaFunction(sdf_image.at<float>(r,c)) * GetPoseDerivatives(r, c, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);
        
        const cv::Mat regularized_depth = GetRegularizedDepth(r,c,current_model);

        for(int i=0;i<pose_derivatives.rows;i++)
          jacobian.at<double>(i,0) += -1 * (region_agreement*pose_derivatives.at<double>(i,0)) + regularized_depth.at<double>(i,0);
        
      }
    }

    ScaleJacobian(jacobian);
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose());

/*** TO DELETE ***/
    cv::Mat canvas = frame_->Mat().clone();
    DrawModelOnFrame(current_model,canvas);
    std::stringstream ss; ss << "step_" << step << ".png";
    cv::imwrite(ss.str(),canvas);

  }

  return current_model.CurrentPose();

}

cv::Mat StereoPWP3D::GetRegularizedDepth(const int r, const int c, const KalmanTracker &current_model) const {
  
  const int NUM_DERIVS = 7;
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  if(front_intersection == cv::Vec3f(0,0,0)) return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
  const double z_estimate = front_intersection[2];
  const double z_stereo = stereo_frame->PtrToDisparityMap()->at<short>(r,c); //scaled by 16?

  const double z_diff = z_estimate - z_stereo;

  //if(z_diff < 1000) {
  //  std::cerr << "estimate = " << z_estimate << "\nstereo = " << z_stereo << "\n\n (" << r << "," << c << ")\n\n\n\n";
  //}
  const double d_mag = 2*(z_estimate - z_stereo)/std::abs(z_estimate - z_stereo);

  cv::Mat x(NUM_DERIVS,1,CV_64FC1);

  for(int dof=0;dof<NUM_DERIVS;dof++){

    const cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);

    x.at<double>(dof,0) = d_mag * dof_derivatives[2];

  }

  return x;

}

const cv::Mat StereoPWP3D::ProjectShapeToSDF(KalmanTracker &current_model) {

  //get the model points at the current pose and project them into the image
  std::vector<SimplePoint<> > points = current_model.ModelPointsAtCurrentPose();
  std::vector<cv::Vec2i > projected_points;
  for(size_t i=0;i<points.size();i++){
    cv::Point2f pt = camera_->rectified_left_eye().ProjectPoint(points[i].vertex_);
    projected_points.push_back( cv::Vec2i( pt.x,pt.y) );
  }

  //find the convex hull of these points
  std::vector<cv::Vec2i> convex_hull;
  cv::convexHull(projected_points,convex_hull);
  cv::Mat convex_hull_(convex_hull);

  //POTENTIAL OPTIMIZATION: find a ROI around the target object to not run tracking over whole image. Not yet implemented.
  FindROI(convex_hull); 

  //find the distance between pixels and the convex hull - heaviside function is applied after this function as need to obtain derivatives of sdf
  cv::Mat sdf_image(ROI_left_.size(),CV_32FC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      //cv::pointpolygontest returns positive inside, negative outside
      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);
    }
  }
  
  return sdf_image;
}

void StereoPWP3D::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model) const {

  cv::Vec3f ray = camera_->rectified_left_eye().UnProjectPoint( cv::Point2i(c,r) ); 
  
  current_model.PtrToModel()->GetIntersection(ray, front_intersection, back_intersection,current_model.CurrentPose());

}

cv::Mat StereoPWP3D::GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model){

  const int NUM_DERIVS = 7;
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);


  if(front_intersection == cv::Vec3f(0,0,0)) return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  
  //std::cout << "front intersection " << cv::Point3f(front_intersection) << std::endl;
  const double z_inv_sq = 1.0/(front_intersection[2]*front_intersection[2]);

  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  for(int dof=0;dof<NUM_DERIVS;dof++){

    const cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);
      
    const double dXdL = camera_->rectified_left_eye().Fx() * (z_inv_sq*(front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2]));
    const double dYdL = camera_->rectified_left_eye().Fy() * (z_inv_sq*(front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2]));
    ret.at<double>(dof,0) = (dSDFdx * dXdL) + (dSDFdy * dYdL);
  
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
