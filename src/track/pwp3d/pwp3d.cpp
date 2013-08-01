#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose){

  //update the translation
  cv::Vec3f translation = jacobian(cv::Range(0,3),cv::Range::all());
  pose.translation_ = pose.translation_ + translation;

  //update the rotation
  sv::Quaternion rotation((float)jacobian.at<double>(3,0),cv::Vec3f((float)jacobian.at<double>(4,0),(float)jacobian.at<double>(5,0),(float)jacobian.at<double>(6,0)));
  pose.rotation_ = pose.rotation_ + rotation;
  pose.rotation_ = pose.rotation_.Normalize();

}

void PWP3D::ScaleJacobian(cv::Mat &jacobian, const int step_number) const {

  // experimental - SCALE THE STEP SIZE?
  static float scales[7] = { (float)1.0 , (float)0.8 , (float)0.7 , (float)0.5, (float)0.4 , (float)0.3, (float)0.1 };

  float scale = (float)3.0;
  //if( step_number > 6) scale = scales[6];
  //else scale = scales[step_number];

  jacobian = (float)0.00000001 * jacobian;
  for(int i=0;i<3;i++) jacobian.at<double>(i,0) *= 150 * scale;
  for(int i=3;i<7;i++) jacobian.at<double>(i,0) *= 0.4 * scale;

}


cv::Vec3f PWP3D::GetDOFDerivatives(const int dof, const Pose &pose, const cv::Vec3f &point_) const {

  //derivatives use the (x,y,z) from the initial reference frame not the transformed one so inverse the transformation
  cv::Vec3f point = point_ - pose.translation_;
  point = pose.rotation_.Inverse().RotateVector(point);

  //return the (dx/dL,dy/dL,dz/dL) derivative for the degree of freedom
  switch(dof){

  case 0: //x
    return cv::Vec3f(1,0,0);
  case 1: //y
    return cv::Vec3f(0,1,0);
  case 2: //z
    return cv::Vec3f(0,0,1);


  case 3: //qw
    return cv::Vec3f((2*pose.rotation_.Y()*point[2])-(2*pose.rotation_.Z()*point[1]),
      (2*pose.rotation_.Z()*point[0])-(2*pose.rotation_.X()*point[2]),
      (2*pose.rotation_.X()*point[1])-(2*pose.rotation_.Y()*point[0]));

  case 4: //qx
    return cv::Vec3f((2*pose.rotation_.Y()*point[1])+(2*pose.rotation_.Z()*point[2]),
      (2*pose.rotation_.Y()*point[0])-(4*pose.rotation_.X()*point[1])-(2*pose.rotation_.W()*point[2]),
      (2*pose.rotation_.Z()*point[0])+(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.X()*point[2]));

  case 5: //qy
    return cv::Vec3f((2*pose.rotation_.X()*point[1])-(4*pose.rotation_.Y()*point[0])+(2*pose.rotation_.W()*point[2]),
      (2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Z()*point[2]),
      (2*pose.rotation_.Z()*point[1])+(2*pose.rotation_.W()*point[0])-(4*pose.rotation_.Y()*point[2]));
  case 6: //qz
    return cv::Vec3f((2*pose.rotation_.X()*point[2])-(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.Z()*point[0]),
      (2*pose.rotation_.W()*point[0])-(4*pose.rotation_.X()*point[1])+(2*pose.rotation_.Y()*point[2]),
      (2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Y()*point[1]));

  default:
    throw std::runtime_error("Error, a value in the range 0-6 must be supplied");
  }
}


double PWP3D::GetEnergy(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const {

  const double pixel_probability = (double)frame_->ClassificationMap().at<unsigned char>(r,c)/255.0;
  const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability/norm;
  const double background_probability = (1-pixel_probability)/norm;
  return -log(Heaviside(sdf)*foreground_probability + (1-Heaviside(sdf))*background_probability);

}

double PWP3D::GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) {
  
  const double pixel_probability = (double)frame_->ClassificationMap().at<unsigned char>(r,c)/255.0;
  const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability/norm;
  const double background_probability = (1-pixel_probability)/norm;
  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf)*foreground_probability + (1.0-Heaviside(sdf))*background_probability);
  return region_agreement;

}


void PWP3D::ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const {
  
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



bool PWP3D::GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model) const {

  cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) ); 
  return current_model.PtrToModel()->GetIntersection(ray, front_intersection, back_intersection,current_model.CurrentPose());

}


bool PWP3D::GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection, const KalmanTracker &current_model) const {

  static const int search_range= 15; //has to be larger than border for search to work!
  if(sdf.at<float>(r,c) <  -10 || sdf.at<float>(r,c) >= 0) return false;
  
  cv::Point2i min_point;
  const cv::Point2i start_pt(c,r);
  float min_dist = std::numeric_limits<float>::max();
  int max_r = r+search_range, min_r = r-search_range, max_c = c+search_range, min_c = c-search_range;
  if(max_r >= sdf.rows) max_r = sdf.rows-1;
  if(max_c >= sdf.cols) max_c = sdf.cols-1;
  if(min_r < 0) min_r = 0;
  if(min_c < 0) min_c = 0;

  for(int row=min_r;row<max_r;row++){
    for(int col=min_c;col<max_c;col++){
      
      if(sdf.at<float>(row,col) < 0.3) continue;
      const cv::Point2i test_pt(col,row);
      const cv::Point2i dist = test_pt - start_pt;
      float dist_val = (float)sqrt((float)dist.x*dist.x + dist.y*dist.y);
      if(dist_val < min_dist){
        min_dist = dist_val;
        min_point = test_pt;
      }
    }
  }

  cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(min_point.x,min_point.y) ); 
  return  current_model.PtrToModel()->GetIntersection(ray, front_intersection,back_intersection,current_model.CurrentPose());
}

cv::Mat PWP3D::GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model){

  const int NUM_DERIVS = 7;
  //if(dSDFdx == 0.0f && dSDFdy == 0.0f) return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
   
   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,current_model);
    if(!intersects)
      return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  }


  if(front_intersection[2] == 0) front_intersection[2] = 0.00000001;
  const double z_inv_sq = 1.0/(front_intersection[2]*front_intersection[2]);
  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  for(int dof=0;dof<NUM_DERIVS;dof++){

    const cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);
      
    const double dXdL = camera_->Fx() * (z_inv_sq*((front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq*((front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2])));
    ret.at<double>(dof,0) = DeltaFunction(sdf.at<float>(r,c)) * ((dSDFdx * dXdL) + (dSDFdy * dYdL));
  
  }


  return ret;
}



const cv::Mat PWP3D::ProjectShapeToSDF(KalmanTracker &current_model) {

  ROI() = frame_->Mat();
  cv::Mat sdf_image(ROI().size(),CV_32FC1);
  cv::Mat intersection_image(ROI().size(),CV_8UC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
      intersection_image.at<unsigned char>(r,c) = 255 * current_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,current_model.CurrentPose());
    }
  }
  
  cv::Mat edge_image(intersection_image.size(),CV_8UC1);
  cv::Canny(intersection_image,edge_image,1,100);
  distanceTransform(~edge_image,sdf_image,CV_DIST_L2,CV_DIST_MASK_PRECISE);
  
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(intersection_image.at<unsigned char>(r,c) != 255)
        sdf_image.at<float>(r,c) *= -1;
    }
  }

  return sdf_image;
  
/*
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

  //POTENTIAL OPTIMIZATION: 
  //find a ROI around the target object to not run tracking over whole image. Not yet implemented.
  FindROI(convex_hull); 
 
  //find the distance between pixels and the convex hull - heaviside function is applied after this function as need to obtain derivatives of sdf
  cv::Mat sdf_image(ROI_left_.size(),CV_32FC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      //cv::pointpolygontest returns positive inside, negative outside
      sdf_image.at<float>(r,c) = (float)pointPolygonTest(convex_hull_,cv::Point2f((float)c,(float)r),true);
    }
  }
  return sdf_image;*/

}


