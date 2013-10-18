#include "../../../headers/track/pwp3d/pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include <boost/math/special_functions/fpclassify.hpp>
using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(const cv::Mat &jacobian, Pose &pose, const int step, const int pixel_count){

  cv::Mat scaled_jacobian;
  jacobian.copyTo(scaled_jacobian);

  ScaleJacobian(scaled_jacobian,step,pixel_count);

  //update the translation
  cv::Vec3f translation = scaled_jacobian(cv::Range(0,3),cv::Range::all());
  pose.translation_ = pose.translation_ + translation;

  //update the rotation
  sv::Quaternion rotation( (float)scaled_jacobian.at<double>(3,0),
                            cv::Vec3f(
                              (float)scaled_jacobian.at<double>(4,0),
                              (float)scaled_jacobian.at<double>(5,0),
                              (float)scaled_jacobian.at<double>(6,0)
                            )
                );
  pose.rotation_ = pose.rotation_ + rotation;
  pose.rotation_ = pose.rotation_.Normalize();

  std::cerr << "Current translation = " << cv::Point3f(pose.translation_) << "\n";

}

void PWP3D::ScaleJacobian(cv::Mat &jacobian, const int step_number, const int pixel_count) const {
    
  static bool swap = true;

  std::cerr << "Pixel count = " << pixel_count << "\n";

  const double SCALE_FACTOR =  (1.0/(pixel_count));
    
  const double XY_scale = 0.2 * 0.005 * swap * SCALE_FACTOR ;
  const double Z_scale = 0.5 * 0.005 * swap * SCALE_FACTOR ;
  const double R_SCALE = 10 * 0.000008 * !swap * SCALE_FACTOR / 25;

  jacobian.at<double>(0,0) *= XY_scale;
  jacobian.at<double>(1,0) *= XY_scale;
  jacobian.at<double>(2,0) *= Z_scale;
  for(int i=3;i<7;i++) jacobian.at<double>(i,0) = R_SCALE * jacobian.at<double>(i,0);

  double T_SIZE = 0;
  for(int i=0;i<3;i++) T_SIZE = jacobian.at<double>(i,0)*jacobian.at<double>(i,0);
  T_SIZE = sqrt(T_SIZE);

  //if(T_SIZE > 5)
  //  for(int i=0;i<3;i++) jacobian.at<double>(i,0)/=T_SIZE;

  double R_SIZE = 0;
  for(int i=3;i<7;i++) R_SIZE = jacobian.at<double>(i,0)*jacobian.at<double>(i,0);
  R_SIZE = sqrt(R_SIZE);

  std::cerr << "R_SIZE = " << R_SIZE << "\n";
  if(R_SIZE > 1.0)
    for(int i=3;i<7;i++) {
      //std::cerr << jacobian.at<double>(i,0) << "\n";
      //jacobian.at<double>(i,0)/=R_SIZE;
      //std::cerr << jacobian.at<double>(i,0) << "\n";
    }
  

  std::cerr << "Jacobian = " << jacobian << "\n";
  swap = !swap;
  return;

  /*static bool swap = false;
  if(!step_number % 5) swap = !swap;

  const int Z_SCALE_FACTOR = 15;

  const double SCALER = 0.9/(1 + exp(-0.15 * (-step_number + 40) )) + 0.1;

  const double T_SCALER = SCALER * 5;
  const double translation_scale = 1.0/(T_SCALER * sqrt( jacobian.at<double>(0,0)*jacobian.at<double>(0,0) + jacobian.at<double>(1,0)*jacobian.at<double>(1,0) + jacobian.at<double>(2,0)*jacobian.at<double>(2,0)));
  for(int i=3;i<7;i++) jacobian.at<double>(i,0) = (float)0.000000002 * jacobian.at<double>(i,0);
  jacobian.at<double>(6,0) *= Z_SCALE_FACTOR;
  
  sv::Quaternion q(boost::math::quaternion<double>(jacobian.at<double>(3,0),jacobian.at<double>(4,0),jacobian.at<double>(5,0),jacobian.at<double>(6,0)));
  const cv::Vec3f angle_axis = q.AngleAxis();
  const double angle = sqrt(angle_axis[0]*angle_axis[0] + angle_axis[1]*angle_axis[1] + angle_axis[2]*angle_axis[2]);
  const double rotation_scale = std::abs(0.8 * SCALER / angle);


  for(int i=0;i<3;i++) jacobian.at<double>(i,0) *= translation_scale * !swap;
  jacobian.at<double>(2,0) *= Z_SCALE_FACTOR ;
  
  jacobian.at<double>(3,0) = rotation_scale * q.W() * swap;
  jacobian.at<double>(4,0) = rotation_scale * q.X() * swap;
  jacobian.at<double>(5,0) = rotation_scale * q.Y() * swap;
  jacobian.at<double>(6,0) = rotation_scale * q.Z() * swap;

  swap = !swap;*/
  
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

  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  //const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  const double foreground_probability = pixel_probability;///norm;
  const double background_probability = (1-pixel_probability);///norm;
  
  double x = (Heaviside(sdf)*foreground_probability + (1-Heaviside(sdf))*background_probability);
  if ( x == std::numeric_limits<double>::infinity() )
    return 1000;  
  return x;

}

double PWP3D::GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) {
  
  const double alpha = 1.0;//0.3; // DEBUGGING TEST !!!!
  const double beta = 1.0;//0.7; // DEBUGGING TEST !!!!

  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  //const double norm = (norm_foreground*pixel_probability) + (norm_background*(1.0-pixel_probability));
  //const double foreground_probability = alpha * (pixel_probability/norm); // DEBUGGING TEST!!!
  //const double background_probability = beta * ((1-pixel_probability)/norm); // DEBUGGING TEST!!! 

  const double foreground_probability = pixel_probability;
  const double background_probability = (1.0 - foreground_probability);

  const double region_agreement = (foreground_probability - background_probability)/ (Heaviside(sdf)*foreground_probability + (1.0-Heaviside(sdf))*background_probability);
  if(region_agreement == std::numeric_limits<double>::infinity())
    return 0.0;
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

  static const int search_range= 40; //has to be larger than border for search to work!
  //if(sdf.at<float>(r,c) <  -10 || sdf.at<float>(r,c) >= 0) return false;
  
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
      
      if(sdf.at<float>(row,col) > 1.5 || sdf.at<float>(row,col) < 0.5) continue; //too far 'inside' or outside - we want teh tangent rays. need to aim for points 1 pixel inside to avoid slight inaccuracies reporting a miss.
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
     
   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  
  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,current_model);
    if(!intersects)
      return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  }


  //just in case...
  if(front_intersection[2] == 0) front_intersection[2] = 0.00000001;
  const double z_inv_sq_front = 1.0/(front_intersection[2]*front_intersection[2]);
  if(back_intersection[2] == 0) back_intersection[2] = 0.00000001;
  const double z_inv_sq_back = 1.0/(back_intersection[2]*back_intersection[2]);

  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  for(int dof=0;dof<NUM_DERIVS;dof++){

    //compute the derivative for each dof
    const cv::Vec3f dof_derivatives_front = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);
    const cv::Vec3f dof_derivatives_back = GetDOFDerivatives(dof,current_model.CurrentPose(),back_intersection);

    const double dXdL = camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2])));
    ret.at<double>(dof,0) = DeltaFunction(sdf.at<float>(r,c)) * ((dSDFdx * dXdL) + (dSDFdy * dYdL));
      
  }
  return ret;

}



const cv::Mat PWP3D::ProjectShapeToSDF(KalmanTracker &current_model) {

  //find all the pixels which project to intersection points on the model
  cv::Mat sdf_image(frame_->GetImageROI().size(),CV_32FC1);
  cv::Mat intersection_image(frame_->GetImageROI().size(),CV_8UC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
      intersection_image.at<unsigned char>(r,c) = 255 * current_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,current_model.CurrentPose());
    }
  }

  //take this binary image and find the outer contour of it. then make a distance image from that contour.
  cv::Mat edge_image(intersection_image.size(),CV_8UC1);
  cv::Canny(intersection_image,edge_image,1,100);
  distanceTransform(~edge_image,sdf_image,CV_DIST_L2,CV_DIST_MASK_PRECISE);
  cv::Mat distance_image = cv::Mat::zeros(sdf_image.size(),CV_8UC1);
  //flip the outside distances so they are negative
  //cv::Mat delta = cv::Mat::zeros(frame_->GetImageROI().size(),CV_8UC1);
  cv::Mat heaviside = cv::Mat::zeros(frame_->GetImageROI().size(),CV_8UC1);
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if(intersection_image.at<unsigned char>(r,c) != 255)
        sdf_image.at<float>(r,c) *= -1;
      //delta.at<unsigned char>(r,c) = 255 * DeltaFunction(sdf_image.at<float>(r,c));
      //heaviside.at<unsigned char>(r,c) = 255 * Heaviside(sdf_image.at<float>(r,c));
      //if(std::abs(sdf_image.at<float>(r,c)) < 32) distance_image.at<unsigned char>(r,c) = 255;
    }
  }
 
  //cv::imwrite("DElta.png",delta);
  return sdf_image;

}


bool PWP3D::ModelInFrame( const KalmanTracker &tracked_model, const cv::Mat &detect_image) const {

  const int rows = detect_image.rows;
  const int cols = detect_image.cols;
  
  size_t tp = 0;
  size_t fp = 0;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(c,r) );
      tp += tracked_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,tracked_model.CurrentPose()) && detect_image.at<unsigned char>(r,c) >= 127;
      fp += tracked_model.PtrToModel()->GetIntersection(ray, cv::Vec3f() , cv::Vec3f() ,tracked_model.CurrentPose()) && detect_image.at<unsigned char>(r,c) < 127;
    }
  }
  
  return ((float)tp / (tp+fp) ) > 0.75;

}


