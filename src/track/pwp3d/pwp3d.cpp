#include <cinder/ImageIo.h>
#include <cinder/app/AppBasic.h>
#include <cinder/Camera.h>
#include <cinder/gl/Light.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <ctime>
#include <CinderOpenCV.h>
#include <cinder/CinderMath.h>

#include "../../../include/track/pwp3d/pwp3d.hpp"
#include "../../../include/utils/helpers.hpp"
#include "../../../include/resources.hpp"
#include "../../../include/constants.hpp"

using namespace ttrk;

PWP3D::PWP3D(const int width, const int height) {

  //need the colour buffer to be 32bit
  ci::gl::Fbo::Format format;
  format.setColorInternalFormat(GL_RGBA32F);
  format.enableColorBuffer(true, 1);
  front_depth_framebuffer_ = ci::gl::Fbo(width, height, format);
 
  //need 2 colour buffers for back contour
  format.enableColorBuffer(true, 2);
  back_depth_framebuffer_ = ci::gl::Fbo(width, height, format);

  LoadShaders();

  HEAVYSIDE_WIDTH = 3;

  NUM_STEPS = 1;

}

PWP3D::~PWP3D(){

  front_depth_framebuffer_.getDepthTexture().setDoNotDispose(false);
  front_depth_framebuffer_.getTexture().setDoNotDispose(false);
  front_depth_framebuffer_.reset();

  back_depth_framebuffer_.getDepthTexture().setDoNotDispose(false);
  back_depth_framebuffer_.getTexture(0).setDoNotDispose(false);
  back_depth_framebuffer_.getTexture(1).setDoNotDispose(false);
  back_depth_framebuffer_.reset();

  //front_depth_.reset();
  //back_depth_and_contour_.reset();

}

void PWP3D::LoadShaders(){

  front_depth_ = ci::gl::GlslProg(ci::app::loadResource(PWP3D_FRONT_DEPTH_VERT), ci::app::loadResource(PWP3D_FRONT_DEPTH_FRAG));
  back_depth_and_contour_ = ci::gl::GlslProg(ci::app::loadResource(PWP3D_BACK_DEPTH_AND_CONTOUR_VERT), ci::app::loadResource(PWP3D_BACK_DEPTH_AND_CONTOUR_FRAG));

}

void PWP3D::UpdateJacobian(const float region_agreement, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, float *jacobian){

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  //compute the derivatives
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_point, 0);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_point, 0);

  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < front_jacs.size(); ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];

    //actually compute the cost function equation for the degree of freedom in question
    float pval = dsdf_dx * (fx * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[0]) - (front_intersection_point[0] * dof_derivatives_front[2]))) + fx * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[0]) - (back_intersection_point[0] * dof_derivatives_back[2]))));
    pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
    pval *= DeltaFunction(sdf);

    jacobian[dof] += region_agreement * pval;

  }

}

bool PWP3D::FindClosestIntersection(const float *sdf_im, const int r, const int c, const int height, const int width, int &closest_r, int &closest_c) const {
  
  const float &sdf_val = sdf_im[r * width + c];
  const int ceil_sdf = (int)std::abs(ceil(sdf_val));
  for (int w_c = c - ceil_sdf; w_c <= c + ceil_sdf; ++w_c){
    
    const int up_idx = (r + ceil_sdf)*width + w_c;
    const int down_idx = (r - ceil_sdf)*width + w_c;
    if (sdf_im[up_idx] >= 0.0){
      closest_r = r + ceil_sdf;
      closest_c = w_c;
      return true;
    }
    else if (sdf_im[down_idx] >= 0.0){
      closest_r = r - ceil_sdf;
      closest_c = w_c;
      return true;
    }
  }

  for (int w_r = r - ceil_sdf; w_r <= r + ceil_sdf; ++w_r){

    const int left_idx = w_r*width + c - ceil_sdf;
    const int right_idx = w_r*width + c + ceil_sdf;
    if (sdf_im[left_idx] >= 0.0){
      closest_r = w_r;
      closest_c = c - ceil_sdf;
      return true;
    }
    else if (sdf_im[right_idx] >= 0.0){
      closest_r = w_r;
      closest_c = c + ceil_sdf;
      return true;
    }
  }
  
  return false;

}

float PWP3D::GetRegionAgreement(const int r, const int c, const float sdf) {
  
  const float pixel_probability = frame_->GetClassificationMapROI().at<float>(r,c);
  const float heaviside_value = HeavisideFunction(sdf);
    
  return (2.0f*pixel_probability - 1.0f)/(heaviside_value*pixel_probability + (1.0f-heaviside_value)*(1.0f-pixel_probability));

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

void PWP3D::RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour){

  assert(front_depth_framebuffer_.getWidth() == camera->Width() && front_depth_framebuffer_.getHeight() == camera->Height());

  //setup camera/transform/matrices etc
  ci::gl::pushMatrices();

  camera->SetupCameraForDrawing();

  ci::gl::enableDepthWrite();
  ci::gl::enableDepthRead();
  glDisable(GL_LIGHTING);

  //// Render front depth 
  front_depth_framebuffer_.bindFramebuffer();
  glClearColor((float)GL_FAR, (float)GL_FAR, (float)GL_FAR, (float)GL_FAR);

  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  ////bind the front depth shader and render
  front_depth_.bind();
  mesh->Render();
  front_depth_.unbind();
 
  front_depth_framebuffer_.unbindFramebuffer();
  glFinish();

  //// Render back depth + contour 
  back_depth_framebuffer_.bindFramebuffer();
  glClearColor((float)GL_FAR, (float)GL_FAR, (float)GL_FAR, (float)GL_FAR);

  glClearDepth(0.0f);
  glDepthFunc(GL_GREATER);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  back_depth_and_contour_.bind();
  back_depth_and_contour_.uniform("tex_w", float(back_depth_framebuffer_.getWidth()));
  back_depth_and_contour_.uniform("tex_h", float(back_depth_framebuffer_.getHeight()));
  back_depth_and_contour_.uniform("far", float(GL_FAR));
  
  ci::gl::Texture tex_fd = front_depth_framebuffer_.getTexture(0);
  tex_fd.enableAndBind();
  back_depth_and_contour_.uniform("tex_fd", 0); //bind it to the current texture
  //back_depth_and_contour_.uniform("tex_cols", 1);
  mesh->Render();
  tex_fd.disable();
  tex_fd.unbind();
  back_depth_and_contour_.unbind();

  back_depth_framebuffer_.unbindFramebuffer();

  //Bring depth test back to normal mode
  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);

  //neccessary to get the results NOW.
  glFinish();

  ci::gl::popMatrices();

  camera->ShutDownCameraAfterDrawing();

  front_depth_framebuffer_.getTexture();
  back_depth_framebuffer_.getTexture();
  back_depth_framebuffer_.getTexture(1);

  cv::Mat front_depth_flipped = ci::toOcv(front_depth_framebuffer_.getTexture());
  cv::Mat back_depth_flipped = ci::toOcv(back_depth_framebuffer_.getTexture(0));
  cv::Mat mcontour = ci::toOcv(back_depth_framebuffer_.getTexture(1));
  cv::Mat fmcontour;
  cv::flip(front_depth_flipped, front_depth, 0);
  cv::flip(back_depth_flipped, back_depth, 0);
  cv::flip(mcontour, fmcontour, 0);

  contour = cv::Mat(mcontour.size(), CV_8UC1);
  float *src = (float *)fmcontour.data;
  unsigned char *dst = (unsigned char*)contour.data;

  for (int r = 0; r < mcontour.rows; ++r){
    for (int c = 0; c < mcontour.cols; ++c){
      dst[r * mcontour.cols + c] = (unsigned char)src[(r * mcontour.cols + c)*4];
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

      if (std::abs(front_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) > EPS){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        front_intersection_image.at<cv::Vec3f>(r, c) = front_depth.at<cv::Vec4f>(r, c)[0]*cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        front_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f((int)GL_FAR, (int)GL_FAR, (int)GL_FAR);
      }
      if (std::abs(back_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) > EPS){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        back_intersection_image.at<cv::Vec3f>(r, c) = back_depth.at<cv::Vec4f>(r, c)[0]*cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        back_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f((int)GL_FAR, (int)GL_FAR, (int)GL_FAR);
      }

    }
  }


  distanceTransform(contour, sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  //flip the sign of the distance image for outside pixels
  for(int r=0;r<sdf_image.rows;r++){
    for(int c=0;c<sdf_image.cols;c++){
      if (std::abs(front_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) < EPS)
        sdf_image.at<float>(r,c) *= -1;
    }
  }
  
}


