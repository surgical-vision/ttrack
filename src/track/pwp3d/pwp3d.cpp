#include <cinder/ImageIo.h>
#include <cinder/app/AppBasic.h>
#include <cinder/Camera.h>
#include <cinder/gl/Light.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <ctime>
#include <CinderOpenCV.h>

#include "../../../include/track/pwp3d/pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"
#include "../../../include/resources.hpp"
#include "../../../include/constants.hpp"

using namespace ttrk;

PWP3D::PWP3D(const int width, const int height) : k_delta_function_std_(2.5), k_heaviside_width_(0.3) {

  //need the colour buffer to be 32bit
  ci::gl::Fbo::Format format;
  format.setColorInternalFormat(GL_RGBA32F);
  format.enableColorBuffer(true, 1);
  front_depth_framebuffer_ = ci::gl::Fbo(width, height, format);
  
  //need 2 colour buffers for back contour
  format.enableColorBuffer(true, 2);
  back_depth_framebuffer_ = ci::gl::Fbo(width, height, format);

  LoadShaders();

}

void PWP3D::LoadShaders(){

  front_depth_ = ci::gl::GlslProg(ci::app::loadResource(PWP3D_FRONT_DEPTH_VERT), ci::app::loadResource(PWP3D_FRONT_DEPTH_FRAG));
  back_depth_and_contour_ = ci::gl::GlslProg(ci::app::loadResource(PWP3D_BACK_DEPTH_AND_CONTOUR_VERT), ci::app::loadResource(PWP3D_BACK_DEPTH_AND_CONTOUR_FRAG));

}

//void PWP3D::ApplyGradientDescentStep(PoseDerivs &jacobian, Pose &pose, const size_t step, const size_t pixel_count){
//
//  ScaleJacobian(jacobian,step,pixel_count);
//
//  //update the translation
//  cv::Vec3d translation(jacobian[0],jacobian[1],jacobian[2]);
//  pose.translation_ = pose.translation_ + translation;
//
//  //update the rotation
//  sv::Quaternion rotation(boost::math::quaternion<double>(jacobian[3],jacobian[4],jacobian[5],jacobian[6]));
//
//  pose.rotation_ = pose.rotation_ + rotation;
//  pose.rotation_ = pose.rotation_.Normalize();
//
//}

//void PWP3D::ScaleJacobian(PoseDerivs &jacobian, const size_t step_number, const size_t pixel_count) const {
//
//  const double SCALE_FACTOR =  (1.0/(pixel_count)) * 3;
//    
//  const double XY_scale = 0.2 * 0.005 * SCALE_FACTOR;
//  const double Z_scale = 0.5 * 0.005 * SCALE_FACTOR;
//  double R_SCALE = 10 * 0.00008 * SCALE_FACTOR;
//  
//  jacobian[0] *= XY_scale;//* 25;
//  jacobian[1] *= XY_scale ;
//  jacobian[2] *= Z_scale; //* camera_->Fx()/4;
//  
//  double largest = std::abs(jacobian[0]);
//  if( largest < std::abs(jacobian[1]) ) largest = std::abs(jacobian[1]);
//  if( largest < std::abs(jacobian[2]) ) largest = std::abs(jacobian[2]);
//
//  jacobian[0] = 0.5 * (jacobian[0]*0.2)/largest;
//  jacobian[1] = 0.5 * (jacobian[1]*0.2)/largest;
//  jacobian[2] = 0.5 * (jacobian[2]*0.4)/largest;
//
//  //jacobian.at<double>(5,0) *= 3;
//
//  largest = std::abs(jacobian[3]);
//  if( largest < std::abs(jacobian[4]) ) largest = std::abs(jacobian[4]);
//  if( largest < std::abs(jacobian[5]) ) largest = std::abs(jacobian[5]);
//  if( largest < std::abs(jacobian[6]) ) largest = std::abs(jacobian[6]);
//
//  for(int i=3;i<7;i++){
//    jacobian[i] *= 0.5 * (0.007 / largest);
//  }
//  
//}

double PWP3D::GetRegionAgreement(const int r, const int c, const double sdf) {
  
  const double pixel_probability = (double)frame_->GetClassificationMapROI().at<unsigned char>(r,c)/255.0;
  const double heaviside_value = Heaviside(sdf, k_heaviside_width_ * BLUR_WIDTH);
    
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

//void PWP3D::GetPoseDerivatives(const int r, const int c, const cv::Mat &sdf, const double dSDFdx, const double dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, PoseDerivs &pd){
//
//   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
//  double front_intersection[3];
//  double back_intersection[3];
//  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,front_intersection_image,back_intersection_image);
//  
//  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
//  if(!intersects) {
//    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,front_intersection_image,back_intersection_image);
//    if(!intersects){
//      pd = PoseDerivs::Zeros();
//      return;
//    }
//  }
//
//#ifdef _DEBUG
//  if( (front_intersection[0] == 0.0 && front_intersection[1] == 0.0 && front_intersection[2] == 0.0) || (back_intersection[0] == 0.0 && back_intersection[1] == 0.0 && back_intersection[2] == 0.0) )
//    throw(std::runtime_error("Error, this is a value we should not get!\n"));
//  if( front_intersection[2] == 0.0 || back_intersection[2] == 0.0 )
//    throw(std::runtime_error("Error, this is a value we should not get!\n"));
//#endif
//
//  const double z_inv_sq_front = 1.0/(front_intersection[2]*front_intersection[2]);
//  const double z_inv_sq_back = 1.0/(back_intersection[2]*back_intersection[2]);
//
//  static double derivs_front[21];
//  static double derivs_back[21];
//  static bool first = true;
//  if(first) {
//    current_model.CurrentPose().SetupFastDOFDerivs(derivs_front);
//    current_model.CurrentPose().SetupFastDOFDerivs(derivs_back);
//    first = false;
//  }
//
//  GetFastDOFDerivs(current_model.CurrentPose(),derivs_front,front_intersection);
//  GetFastDOFDerivs(current_model.CurrentPose(),derivs_back,back_intersection);
//  
//  for(int dof=0;dof<PoseDerivs::NUM_VALS;dof++){
//    
//    //compute the derivative for each dof
//    const double *dof_derivatives_front = derivs_front+(3*dof);
//    const double *dof_derivatives_back = derivs_back+(3*dof);
//
//    pd[dof] = dSDFdx * (camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2]))));
//    pd[dof] += dSDFdy * (camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2]))));
//    pd[dof] *= DeltaFunction(sdf.at<float>(r,c), k_delta_function_std_ * blurring_scale_factor_ );
//  
//  }
//  
//}

void PWP3D::RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour){

  assert(front_depth_framebuffer_.getWidth() == camera->Width() && front_depth_framebuffer_.getHeight() == camera->Height());

  //setup camera/transform/matrices etc
  ci::gl::pushMatrices();
  camera->SetupCameraForDrawing(front_depth_framebuffer_.getWidth(), front_depth_framebuffer_.getHeight());

  ci::gl::enableDepthWrite();
  ci::gl::enableDepthRead();

  // Render front depth 
  front_depth_framebuffer_.bindFramebuffer();
  glViewport(0, 0, front_depth_framebuffer_.getWidth(), front_depth_framebuffer_.getHeight());
  glScissor(0, 0, front_depth_framebuffer_.getWidth(), front_depth_framebuffer_.getHeight());
  glClearColor(GL_FAR, GL_FAR, GL_FAR, GL_FAR);

  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //bind the front depth shader and render
  front_depth_.bind();
  mesh->Render();
  front_depth_.unbind();
  
  front_depth_framebuffer_.unbindFramebuffer();
  glFinish();

  // Render back depth + contour 
  back_depth_framebuffer_.bindFramebuffer();
  glViewport(0, 0, back_depth_framebuffer_.getWidth(), back_depth_framebuffer_.getHeight());
  glScissor(0, 0, back_depth_framebuffer_.getWidth(), back_depth_framebuffer_.getHeight());
  glClearColor(GL_FAR, GL_FAR, GL_FAR, GL_FAR);

  glClearDepth(0.0f);
  glDepthFunc(GL_GREATER);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  back_depth_and_contour_.bind();
  back_depth_and_contour_.uniform("tex_w", float(back_depth_framebuffer_.getWidth()));
  back_depth_and_contour_.uniform("tex_h", float(back_depth_framebuffer_.getHeight()));
  back_depth_and_contour_.uniform("far", GL_FAR);
  
  ci::gl::Texture tex_fd = front_depth_framebuffer_.getTexture(0);
  tex_fd.enableAndBind();
  back_depth_and_contour_.uniform("tex_fd", 0); //bind it to the current texture
   
  mesh->Render();

  tex_fd.unbind();
  
  back_depth_and_contour_.unbind();

  back_depth_framebuffer_.unbindFramebuffer();

  //Bring depth test back to normal mode
  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);

  glPopMatrix();
  //neccessary to get the results NOW.
  glFinish();

  front_depth = ci::toOcv(front_depth_framebuffer_.getTexture());
  back_depth = ci::toOcv(back_depth_framebuffer_.getTexture(0));
  cv::Mat mcontour = ci::toOcv(back_depth_framebuffer_.getTexture(1));
  contour = cv::Mat(mcontour.size(), CV_8UC1);
  float *src = (float *)mcontour.data;
  unsigned char *dst = (unsigned char*)contour.data;

  for (int r = 0; r < mcontour.rows; ++r){
    for (int c = 0; c < mcontour.cols; ++c){
      dst[r * mcontour.cols + c] = src[(r * mcontour.cols + c)*4];
    }
  }

}

void PWP3D::ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  sdf_image = cv::Mat(frame_->GetImageROI().size(),CV_32FC1);
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(),CV_32FC3);


  cv::Mat front_depth, back_depth, contour;
  RenderModelForDepthAndContour(mesh, camera, front_depth, back_depth, contour );
  
  cv::Mat unprojected_image_plane = camera->GetUnprojectedImagePlane(front_intersection_image.cols, front_intersection_image.rows);

  for (int r = 0; r < front_intersection_image.rows; r++){
    for (int c = 0; c < front_intersection_image.cols; c++){

      if (front_depth.at<cv::Vec4f>(r, c)[0] != 0){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        front_intersection_image.at<cv::Vec3f>(r, c) = front_depth.at<float>(r, c)*cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        front_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f(GL_FAR, GL_FAR, GL_FAR);
      }
      if (back_depth.at<cv::Vec4f>(r, c)[0] != GL_FAR){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        back_intersection_image.at<cv::Vec3f>(r, c) = back_depth.at<float>(r, c)*cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        back_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f(GL_FAR, GL_FAR, GL_FAR);
      }

    }
  }


  distanceTransform(contour, sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  //flip the sign of the distance image for outside pixels
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if (front_depth.at<float>(r, c) == GL_FAR)
        sdf_image.at<float>(r,c) *= -1;
    }
  }
  
}


