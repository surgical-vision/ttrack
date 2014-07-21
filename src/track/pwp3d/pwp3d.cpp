#include "../../../include/track/pwp3d/pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"
#include <boost/math/special_functions/fpclassify.hpp>
#include "../../../include/utils/renderer.hpp"
#include <cinder/ImageIo.h>
#include <cinder/app/AppBasic.h>


#include <ctime>

using namespace ttrk;

void PWP3D::ApplyGradientDescentStep(PoseDerivs &jacobian, Pose &pose, const size_t step, const size_t pixel_count){

  ScaleJacobian(jacobian,step,pixel_count);

  //update the translation
  cv::Vec3d translation(jacobian[0],jacobian[1],jacobian[2]);
  pose.translation_ = pose.translation_ + translation;

  //update the rotation
  sv::Quaternion rotation(boost::math::quaternion<double>(jacobian[3],jacobian[4],jacobian[5],jacobian[6]));

  pose.rotation_ = pose.rotation_ + rotation;
  pose.rotation_ = pose.rotation_.Normalize();

}

void PWP3D::ScaleJacobian(PoseDerivs &jacobian, const size_t step_number, const size_t pixel_count) const {

  const double SCALE_FACTOR =  (1.0/(pixel_count)) * 3;
    
  const double XY_scale = 0.2 * 0.005 * SCALE_FACTOR;
  const double Z_scale = 0.5 * 0.005 * SCALE_FACTOR;
  double R_SCALE = 10 * 0.00008 * SCALE_FACTOR;
  
  jacobian[0] *= XY_scale;//* 25;
  jacobian[1] *= XY_scale ;
  jacobian[2] *= Z_scale; //* camera_->Fx()/4;
  
  double largest = std::abs(jacobian[0]);
  if( largest < std::abs(jacobian[1]) ) largest = std::abs(jacobian[1]);
  if( largest < std::abs(jacobian[2]) ) largest = std::abs(jacobian[2]);

  jacobian[0] = 0.5 * (jacobian[0]*0.2)/largest;
  jacobian[1] = 0.5 * (jacobian[1]*0.2)/largest;
  jacobian[2] = 0.5 * (jacobian[2]*0.4)/largest;

  //jacobian.at<double>(5,0) *= 3;

  largest = std::abs(jacobian[3]);
  if( largest < std::abs(jacobian[4]) ) largest = std::abs(jacobian[4]);
  if( largest < std::abs(jacobian[5]) ) largest = std::abs(jacobian[5]);
  if( largest < std::abs(jacobian[6]) ) largest = std::abs(jacobian[6]);

  for(int i=3;i<7;i++){
    jacobian[i] *= 0.5 * (0.007 / largest);
  }
  
}


double PWP3D::GetRegionAgreement(const int r, const int c, const double sdf) {
  
  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  const double heaviside_value = Heaviside(sdf, k_heaviside_width_ * blurring_scale_factor_);
    
#ifdef _DEBUG
  const double region_agreement =  (2*pixel_probability - 1)/(heaviside_value*pixel_probability + (1.0-heaviside_value)*(1-pixel_probability));
  if(region_agreement == std::numeric_limits<double>::infinity())
    return 0.0;
#endif
  return (2*pixel_probability - 1)/(heaviside_value*pixel_probability + (1.0-heaviside_value)*(1-pixel_probability));

}

bool PWP3D::GetTargetIntersections(const int r, const int c, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const {

#ifdef _DEBUG 
  if(front_intersection_image.type() != CV_64FC3) throw(std::runtime_error("Error, the type of the front intersection matrix should be 3xfloat\n"));
  if(back_intersection_image.type() != CV_64FC3) throw(std::runtime_error("Error, the type of the back intersection matrix should be 3xfloat\n"));
#endif

  const int index = (r*front_intersection_image.cols + c)*3;
  memcpy(front_intersection, ((double *)front_intersection_image.data)+index, 3*sizeof(double));
  memcpy(back_intersection, ((double *)back_intersection_image.data)+index, 3*sizeof(double));

  return !(front_intersection[0] == 0.0 && front_intersection[1] == 0.0 && front_intersection[2] == 0.0); 

}

bool PWP3D::GetNearestIntersection(const int r, const int c, const cv::Mat &sdf, double *front_intersection, double *back_intersection, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image) const {

  static const int search_range= 20; //has to be larger than border for search to work!

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

  if(min_dist == std::numeric_limits<float>::max()) return false;

  return GetTargetIntersections(min_point.y,min_point.x,front_intersection,back_intersection,front_intersection_image,back_intersection_image);
}

void PWP3D::GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const double dSDFdx, const double dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, PoseDerivs &pd){

   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  double front_intersection[3];
  double back_intersection[3];
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,front_intersection_image,back_intersection_image);
  
  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,front_intersection_image,back_intersection_image);
    if(!intersects){
      pd = PoseDerivs::Zeros();
      return;
    }
  }

#ifdef _DEBUG
  if( (front_intersection[0] == 0.0 && front_intersection[1] == 0.0 && front_intersection[2] == 0.0) || (back_intersection[0] == 0.0 && back_intersection[1] == 0.0 && back_intersection[2] == 0.0) )
    throw(std::runtime_error("Error, this is a value we should not get!\n"));
  if( front_intersection[2] == 0.0 || back_intersection[2] == 0.0 )
    throw(std::runtime_error("Error, this is a value we should not get!\n"));
#endif

  const double z_inv_sq_front = 1.0/(front_intersection[2]*front_intersection[2]);
  const double z_inv_sq_back = 1.0/(back_intersection[2]*back_intersection[2]);

  static double derivs_front[21];
  static double derivs_back[21];
  static bool first = true;
  if(first) {
    current_model.CurrentPose().SetupFastDOFDerivs(derivs_front);
    current_model.CurrentPose().SetupFastDOFDerivs(derivs_back);
    first = false;
  }

  GetFastDOFDerivs(current_model.CurrentPose(),derivs_front,front_intersection);
  GetFastDOFDerivs(current_model.CurrentPose(),derivs_back,back_intersection);
  
  for(int dof=0;dof<PoseDerivs::NUM_VALS;dof++){
    
    //compute the derivative for each dof
    const double *dof_derivatives_front = derivs_front+(3*dof);
    const double *dof_derivatives_back = derivs_back+(3*dof);

    pd[dof] = dSDFdx * (camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2]))));
    pd[dof] += dSDFdy * (camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2]))));
    pd[dof] *= DeltaFunction(sdf.at<float>(r,c), k_delta_function_std_ * blurring_scale_factor_ );
  
  }
  
}

void PWP3D::GetSDFAndIntersectionImage(KalmanTracker &current_model, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  sdf_image = cv::Mat(frame_->GetImageROI().size(),CV_32FC1);
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_64FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_64FC3);
 
  //blocks here
  cv::Mat canvas,z_buffer,binary_image;
  Renderer::DrawMesh(current_model.PtrToModel(), canvas, z_buffer, binary_image, current_model.CurrentPose(), camera_);
 
  cv::Mat unprojected_image_plane = camera_->GetUnprojectedImagePlane(front_intersection_image.cols, front_intersection_image.rows);
  //find the set of pixels which correspond to the drawn object and create the intersection image

  for (int r = 0; r < front_intersection_image.rows; r++){
    for (int c = 0; c < front_intersection_image.cols; c++){
      const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
      front_intersection_image.at<cv::Vec3d>(r, c) = z_buffer.at<float>(r, c)*cv::Vec3d(unprojected_pixel[0], unprojected_pixel[1], 1);
    }
  }

  back_intersection_image = front_intersection_image.clone(); 

  //take this binary image and find the outer contour of it. then make a distance image from that contour.
  cv::Mat edge_image(binary_image.size(), CV_8UC1);
  cv::Canny(binary_image, edge_image, 1, 100);
  std::vector<std::vector<cv::Point> > output_contours;
  //std::vector<std::vector<int> > heirarchy;
  cv::findContours(edge_image, output_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //CV_RETR_EXTERNAL only returns the outer contours
  cv::Mat edge_image_no_inner = cv::Mat::zeros(edge_image.size(), CV_8UC1);
  
  //a shitty hack to clear out some islands in the center of the contour. no idea why the come out of the z buffer.
  for (auto contour = output_contours.begin(); contour != output_contours.end(); ++contour){
    if (contour->size() < 100) continue;
    std::vector<std::vector<cv::Point> > single_contour;
    single_contour.push_back(*contour);
    cv::drawContours(edge_image_no_inner, single_contour, -1, cv::Scalar(255, 255, 255));
  }
  distanceTransform(~edge_image_no_inner, sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  //flip the sign of the distance image for outside pixels
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if (binary_image.at<unsigned char>(r, c) != 255)
        sdf_image.at<float>(r,c) *= -1;
    }
  }

  cv::imwrite("../../canvas.png", canvas);
  cv::imwrite("../../binary_image.png", binary_image);
  cv::imwrite("../../edge_image.png", edge_image_no_inner);
  cv::imwrite("../../sdf_image.png", sdf_image);
  
}


