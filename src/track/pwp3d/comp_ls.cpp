#include <cinder/Camera.h>
#include <cinder/gl/Light.h>
#include <CinderOpenCV.h>
#include <cinder/CinderMath.h>

#include "../../../include/ttrack/track/pwp3d/comp_ls.hpp"
#include "../../../include/ttrack/resources.hpp"
#include "../../../include/ttrack/constants.hpp"

using namespace ttrk;

ComponentLevelSet::ComponentLevelSet(boost::shared_ptr<StereoCamera> camera) : StereoPWP3D(camera) {

  ci::gl::Fbo::Format format;
  format.setColorInternalFormat(GL_RGBA32F);
  format.enableColorBuffer(true, 1);
  component_map_framebuffer_ = ci::gl::Fbo(camera->left_eye()->Width(), camera->left_eye()->Height(), format);

  LoadShaders();

  HomogenousComponent background(0);
  HomogenousComponent shaft(1);
  HomogenousComponent head(2);
  components_.push_back(background);
  components_.push_back(shaft);
  components_.push_back(head);

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
  
  cv::Mat front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, front_intersection_image, back_intersection_image);
  
  //if (!point_registration_){
  //  point_registration_.reset(new PointRegistration(stereo_camera_->left_eye()));
  //  point_registration_->ComputeDescriptorsForPointTracking(frame_, front_intersection_image, current_model->GetBasePose());
  //}

  float fg_area = 1, bg_area = 1;
  
  for (size_t comp = 0; comp < components_.size(); ++comp){

    cv::Mat &sdf_image = components_[comp].sdf_image;

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c){

        const int i = r*classification_image.cols + c;

        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

          if (std::abs(sdf_im_data[i]) > 5){
            throw std::runtime_error("Err");
          }

          //-log(H * P_f + (1-H) * P_b)
          //error += GetErrorValue(classification_image, r, c, sdf_im_data[i], components_[comp].target_probability, fg_area, bg_area);

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], components_[comp].target_probability, ComputeNearestNeighbourForPixel(r, c, sdf_im_data[i], classification_image.rows, classification_image.cols));

          if (std::abs(sdf_im_data[i]) > 5){
            throw std::runtime_error("Err");
          }
          int shifted_i = i;

          ci::app::console() << "i = " << i << std::endl;
          ci::app::console() << "SDF IMAGE DATA = " << sdf_im_data[i] << std::endl;

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
  }

  /*std::vector<MatchedPair> pnp_pairs;
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
*/
}

unsigned char ComponentLevelSet::ComputeNearestNeighbourForPixel(const int r, const int c, const float sdf_value, const int width, const int height){

  const unsigned char pixel_class = component_map_.at<unsigned char>(r, c);

  const unsigned char *comp_map_data = component_map_.data;

  const int ceil_sdf = (int)std::abs(ceil(sdf_value)) + 1;
  for (int w_c = c - ceil_sdf; w_c <= c + ceil_sdf; ++w_c){

    const int up_idx = (r + ceil_sdf)*width + w_c;
    const int down_idx = (r - ceil_sdf)*width + w_c;
    if (comp_map_data[up_idx] != pixel_class){
      return comp_map_data[up_idx];
    }
    else if (comp_map_data[down_idx] != pixel_class){
      return comp_map_data[down_idx];
    }
  }

  for (int w_r = r - ceil_sdf; w_r <= r + ceil_sdf; ++w_r){

    const int left_idx = w_r*width + c - ceil_sdf;
    const int right_idx = w_r*width + c + ceil_sdf;
    if (comp_map_data[left_idx] != pixel_class){
      return comp_map_data[left_idx];
    }
    else if (comp_map_data[right_idx] != pixel_class){
      return comp_map_data[right_idx];
    }

  }

  return false;

}

float ComponentLevelSet::GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_probability, const size_t neighbour_probability){

  float *c_data = (float *)classification_image.data;
  const float pixel_probability = c_data[(classification_image.cols * r + c) * classification_image.channels() + target_probability];
  const float heaviside_value = HeavisideFunction(sdf_value);

  const float Pf = pixel_probability;
  const float Pb = neighbour_probability;

  return (Pf - Pb) / (((heaviside_value*Pf) + ((1 - heaviside_value)*Pb)) + EPS);

}

void ComponentLevelSet::ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);

  cv::Mat front_depth, back_depth;
  RenderModelForDepthAndContour(mesh, camera, front_depth, back_depth);

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


  //cv::Mat sdf_image;
  //distanceTransform(~component_contour_image, component_sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  for (size_t i = 0; i < components_.size(); i++){
    components_[i].sdf_image = cv::Mat(front_depth.size(), CV_32FC1);
    distanceTransform(~components_[i].contour_image, components_[i].sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);
  }

  for (int r = 0; r < front_depth.rows; ++r){
    for (int c = 0; c < front_depth.cols; ++c){
      for (size_t i = 0; i < components_.size(); i++){
        if (components_[i].target_probability != component_map_.at<unsigned char>(r, c)){
          components_[i].sdf_image.at<float>(r, c) *= -1;
        }
      }
    }
  }

  cv::Mat &a1 = components_[0].sdf_image;
  cv::Mat &a2 = components_[1].sdf_image;
  cv::Mat &a3 = components_[2].sdf_image;

}

inline bool IsGreaterThanNeighbour(const cv::Mat &im, const int r, const int c){

  const unsigned char v = im.at<unsigned char>(r, c);
  return v > im.at<unsigned char>(r + 1, c) || v > im.at<unsigned char>(r - 1, c) || v > im.at<unsigned char>(r, c + 1) || v > im.at<unsigned char>(r, c - 1); 
  
}

void ComponentLevelSet::RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth){

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

  //bind the component shader and render
  component_shader_.bind();

  component_shader_.uniform("tex0", 0);
  mesh->RenderTexture(0);
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

  cv::Mat front_depth_flipped = ci::toOcv(front_depth_framebuffer_.getTexture());
  cv::Mat back_depth_flipped = ci::toOcv(back_depth_framebuffer_.getTexture(0));

  cv::flip(front_depth_flipped, front_depth, 0);
  cv::flip(back_depth_flipped, back_depth, 0);

  cv::Mat f_component_map = ci::toOcv(component_map_framebuffer_.getTexture());
  cv::flip(f_component_map, f_component_map, 0);

  float *comp_src = (float *)f_component_map.data;
  component_map_ = cv::Mat(f_component_map.size(), CV_8UC1);
  unsigned char *comp_dst = (unsigned char *)component_map_.data;

  //get the binary component images and the component map.
  for (size_t i = 0; i < components_.size(); ++i){
    components_[i].binary_image = cv::Mat::zeros(front_depth_flipped.size(), CV_8UC1);
    components_[i].contour_image = cv::Mat::zeros(front_depth_flipped.size(), CV_8UC1);
  }
  for (int r = 0; r < front_depth_flipped.rows; ++r){
    for (int c = 0; c < front_depth_flipped.cols; ++c){
      if (std::abs(comp_src[(r * front_depth_flipped.cols + c) * 4]) < EPS){
        comp_dst[r * front_depth_flipped.cols + c] = 1; //plastic shaft
        components_[1].binary_image.at<unsigned char>(r, c) = 255;
      }
      else if (std::abs(comp_src[(r * front_depth_flipped.cols + c) * 4] - 0.686) < 0.01){
        comp_dst[r * front_depth_flipped.cols + c] = 2; //metal head
        components_[2].binary_image.at<unsigned char>(r, c) = 255;
      }
      else{
        comp_dst[r * front_depth_flipped.cols + c] = 0; //background
        components_[0].binary_image.at<unsigned char>(r, c) = 255;
      }
    }
  }

  //create the contour map from the 

  for (int r = 1; r < front_depth_flipped.rows - 1; ++r){
    for (int c = 1; c < front_depth_flipped.cols - 1; ++c){
      for (int i = 0; i < components_.size(); ++i){
        if (IsGreaterThanNeighbour(components_[i].binary_image, r, c)){
          components_[i].contour_image.at<unsigned char>(r, c) = 255;
        }
      }
    }
  }


}