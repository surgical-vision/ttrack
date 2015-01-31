#include <cinder/Camera.h>
#include <cinder/gl/Light.h>
#include <CinderOpenCV.h>
#include <cinder/CinderMath.h>

#include "../../../include/track/pwp3d/comp_ls.hpp"
#include "../../../include/resources.hpp"
#include "../../../include/constants.hpp"

using namespace ttrk;

ComponentLevelSet::ComponentLevelSet(boost::shared_ptr<StereoCamera> camera) : StereoPWP3D(camera) {

  ci::gl::Fbo::Format format;
  format.setColorInternalFormat(GL_RGBA32F);
  format.enableColorBuffer(true, 1);
  component_map_framebuffer_ = ci::gl::Fbo(camera->left_eye()->Width(), camera->left_eye()->Height(), format);

  LoadShaders();

}

ComponentLevelSet::~ComponentLevelSet(){

  component_map_framebuffer_.getDepthTexture().setDoNotDispose(false);
  component_map_framebuffer_.getTexture().setDoNotDispose(false);
  component_map_framebuffer_.reset();

}

void ComponentLevelSet::LoadShaders(){

  component_shader_ = ci::gl::GlslProg(ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_VERT), ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_FRAG));

}

void ComponentLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){
  
  cv::Mat sdf_image, front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, sdf_image, front_intersection_image, back_intersection_image);

  if (!point_registration_){
    point_registration_.reset(new PointRegistration(stereo_camera_->left_eye()));
    point_registration_->ComputeDescriptorsForPointTracking(frame_, front_intersection_image, current_model->GetBasePose());
  }

  //size_t fg_area, bg_area = 0;
  //size_t contour_area = 0;
  //ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;


  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //-log(H * P_f + (1-H) * P_b)
        error += GetErrorValue(classification_image, r, c, sdf_im_data[i], 1.0f);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i]);

        int shifted_i = i;

        //find the closest point on the contour if this point is outside the contour
        if (sdf_im_data[i] < 0.0) {
          int closest_r, closest_c;
          bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
          if (!found) continue; //should this be allowed to happen?
          shifted_i = closest_r * sdf_image.cols + closest_c;
        }

        cv::Matx<float, 1, 7> jacs;
        for (int j = 0; j < 7; ++j){
          jacs(j) = 0.0f;
        }

        const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
        const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

        //update the jacobian
        if (camera == stereo_camera_->left_eye())
          UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else if (camera == stereo_camera_->right_eye())
          UpdateJacobianRightEye(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);
        else
          throw std::runtime_error("");

        jacobian += jacs.t();
        hessian_approx += (jacs.t() * jacs);

      }
    }
  }

  //ci::app::console() << "Before Jacobian = [";
  //for (int i = 0; i < 7; ++i)
  //  ci::app::console() << jacobian(i) << ", ";
  //ci::app::console() << "]" << std::endl;

  std::vector<MatchedPair> pnp_pairs;
  point_registration_->FindPointCorrespondencesWithPose(frame_, current_model, current_model->GetBasePose(), pnp_pairs);

  cv::Matx<float, 1, 7> points_jacobian = cv::Matx<float, 1, 7>::zeros();

  for (auto pnp = pnp_pairs.begin(); pnp != pnp_pairs.end(); pnp++){
    std::vector<float> point_jacobian = point_registration_->GetPointDerivative(pnp->learned_point, cv::Point2f(pnp->image_point[0], pnp->image_point[1]), current_model->GetBasePose());

    for (int i = 0; i < jacobian.rows; ++i){
      points_jacobian(i) += 0.0005 * point_jacobian[i];
    }

    jacobian += points_jacobian.t();
    hessian_approx += (points_jacobian.t() * points_jacobian);
  }

}

void ComponentLevelSet::ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  sdf_image = cv::Mat(frame_->GetImageROI().size(), CV_32FC1);
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);

  cv::Mat front_depth, back_depth, contour;
  RenderModelForDepthAndContour(mesh, camera, front_depth, back_depth, contour);

  cv::Mat unprojected_image_plane = camera->GetUnprojectedImagePlane(front_intersection_image.cols, front_intersection_image.rows);

  for (int r = 0; r < front_intersection_image.rows; r++){
    for (int c = 0; c < front_intersection_image.cols; c++){

      if (std::abs(front_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) > EPS){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        front_intersection_image.at<cv::Vec3f>(r, c) = front_depth.at<cv::Vec4f>(r, c)[0] * cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        front_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f((int)GL_FAR, (int)GL_FAR, (int)GL_FAR);
      }
      if (std::abs(back_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) > EPS){
        const cv::Vec2f &unprojected_pixel = unprojected_image_plane.at<cv::Vec2f>(r, c);
        back_intersection_image.at<cv::Vec3f>(r, c) = back_depth.at<cv::Vec4f>(r, c)[0] * cv::Vec3f(unprojected_pixel[0], unprojected_pixel[1], 1);
      }
      else{
        back_intersection_image.at<cv::Vec3f>(r, c) = cv::Vec3f((int)GL_FAR, (int)GL_FAR, (int)GL_FAR);
      }

    }
  }


  distanceTransform(contour, sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  //flip the sign of the distance image for outside pixels
  for (int r = 0; r<sdf_image.rows; r++){
    for (int c = 0; c<sdf_image.cols; c++){
      if (std::abs(front_depth.at<cv::Vec4f>(r, c)[0] - GL_FAR) < EPS)
        sdf_image.at<float>(r, c) *= -1;
    }
  }

}

void ComponentLevelSet::RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour){

  assert(front_depth_framebuffer_.getWidth() == camera->Width() && front_depth_framebuffer_.getHeight() == camera->Height());

  //setup camera/transform/matrices etc
  ci::gl::pushMatrices();

  camera->SetupCameraForDrawing();

  ci::gl::enableDepthWrite();
  ci::gl::enableDepthRead();
  glDisable(GL_LIGHTING);

  // Render front depth 
  front_depth_framebuffer_.bindFramebuffer();
  glClearColor((float)GL_FAR, (float)GL_FAR, (float)GL_FAR, (float)GL_FAR);

  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //bind the front depth shader and render
  front_depth_.bind();
  mesh->RenderMaterial();
  front_depth_.unbind();

  front_depth_framebuffer_.unbindFramebuffer();
  glFinish();

  //render the component map
  component_map_framebuffer_.bindFramebuffer();
  glClearColor((float)GL_FAR, (float)GL_FAR, (float)GL_FAR, (float)GL_FAR);

  glClearDepth(1.0f);
  glDepthFunc(GL_LESS);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //bind the front depth shader and render
  component_shader_.bind();
  mesh->RenderMaterial();
  component_shader_.unbind();

  component_map_framebuffer_.unbindFramebuffer();
  glFinish();

  // Render back depth + contour 
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
  
  mesh->RenderMaterial();
  
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
      dst[r * mcontour.cols + c] = (unsigned char)src[(r * mcontour.cols + c) * 4];
    }
  }

}