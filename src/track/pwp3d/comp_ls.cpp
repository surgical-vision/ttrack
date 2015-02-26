#include <cinder/Camera.h>
#include <cinder/gl/Light.h>
#include <CinderOpenCV.h>
#include <cinder/CinderMath.h>

#include "../../../include/ttrack/track/pwp3d/comp_ls.hpp"
#include "../../../include/ttrack/resources.hpp"
#include "../../../include/ttrack/constants.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

#define GRAD_DESCENT

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


void ComponentLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;

  cv::Mat front_intersection_image, back_intersection_image, front_normal_image;
  ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), front_intersection_image, back_intersection_image, front_normal_image);


  if (curr_step == NUM_STEPS) {

    curr_step = 0;
    
    auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
    
    if (!lk_tracker_){

      lk_tracker_.reset(new LKTracker(stereo_camera_->left_eye()));
      lk_tracker_->SetFrontIntersectionImage(front_intersection_image);
      lk_tracker_->InitializeTracker(stereo_frame->GetLeftImage(), current_model->GetBasePose());

    }
    else{

      lk_tracker_->SetFrontIntersectionImage(front_intersection_image);
      lk_tracker_->TrackLocalPoints(stereo_frame->GetLeftImage());

    }

  }

  ++curr_step;

  //float left_error = DoRegionBasedAlignmentStepForLeftEye(current_model);
  //float right_error = DoRegionBasedAlignmentStepForRightEye(current_model);
  //float point_error = DoPointBasedAlignmentStepForLeftEye(current_model);
  //UpdateWithErrorValue(left_error + right_error + point_error);
  //errors_.push_back(left_error + right_error + point_error);

  float error = DoAlignmentStep(current_model);
  UpdateWithErrorValue(error);
  errors_.push_back(error);

  
}

float ComponentLevelSet::DoAlignmentStep(boost::shared_ptr<Model> current_model){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> hessian_approx = cv::Matx<float, 7, 7>::zeros();
  ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), jacobian, hessian_approx, error);
  ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), jacobian, hessian_approx, error);
  ComputePointRegistrationJacobian(current_model, jacobian, hessian_approx);

#ifdef GRAD_DESCENT

  std::vector<float> jacs = ScaleRigidJacobian(jacobian);
  current_model->UpdatePose(jacs);

#else

  jacobian = hessian_approx.inv() * jacobian;
  std::vector<float> jacs(7, 0);
  for (size_t v = 0; v < 7; ++v){
    jacs[v] = -jacobian(v);
  }
  current_model->UpdatePose(jacs);

#endif

  lk_tracker_->UpdatePointsOnModelAfterDerivatives(current_model->GetBasePose());
  
  return error;

}

void ComponentLevelSet::LoadShaders(){

  component_shader_ = ci::gl::GlslProg(ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_VERT), ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_FRAG));

}

void ComponentLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error){
  

  cv::Mat front_intersection_image, back_intersection_image, front_normal_image;

  ProcessSDFAndIntersectionImage(current_model, camera, front_intersection_image, back_intersection_image, front_normal_image);
  
  for (size_t comp = 1; comp < components_.size(); ++comp){

    cv::Mat &sdf_image = components_[comp].sdf_image;

    //SaveSDFImage(sdf_image, "z:/sdf_image.png");

    cv::Mat target_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat nearest_neighbour_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat region_agreement_im = cv::Mat::zeros(sdf_image.size(), CV_32FC1);

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c){

        const int i = r*classification_image.cols + c;

        assert(sdf_image.at<float>(r, c) == sdf_im_data[i]);

        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){
         
          //get the target label for this classification
          size_t target_label = components_[comp].target_probability;

          //find the nearest neighbouring pixel with a different label to this one - if we are inside the contour then search for the nearest different label, if we are outside the contour (i.e. looking at 'background') then just choose the pixel.
          size_t nearest_different_neighbour_label;
          if (sdf_im_data[i] >= 0){
            nearest_different_neighbour_label = ComputeNearestNeighbourForPixel(r, c, sdf_im_data[i], classification_image.cols, classification_image.rows);
          }
          else{
            nearest_different_neighbour_label = component_map_.at<unsigned char>(r, c);
          }
          
          if (nearest_different_neighbour_label == 255){
            target_label_image.at<unsigned char>(r, c) = 255;
            continue;
          }
                    
          error += GetErrorValue(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);
          region_agreement_im.at<float>(r, c) = region_agreement;

          int shifted_i = i;
          
          //find the closest point on the contour if this point is outside the contour
          if (sdf_im_data[i] < 0.0) {
            if (nearest_different_neighbour_label == 0){
              int closest_r, closest_c;
              bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
              if (!found) continue; //should this be allowed to happen?
              shifted_i = closest_r * sdf_image.cols + closest_c;
            }
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

  for (int i = 0; i < 7; ++i){
    ci::app::console() << "region jacobian[" << i << "] = " << jacobian(i) << std::endl;
  }
}

unsigned char ComponentLevelSet::ComputeNearestNeighbourForPixel(const int r, const int c, const float sdf_value, const int width, const int height){

  cv::Mat &component_map_m = component_map_;

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
  
  return 255;

}

float ComponentLevelSet::GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_label, const size_t neighbour_label){

  cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);
  
  float pixel_probability = re[target_label]; //
  float neighbour_probability = re[neighbour_label];
  
  const float heaviside_value = HeavisideFunction(sdf_value);

  if (SAFE_EQUALS(pixel_probability, 0.0f) && SAFE_EQUALS(neighbour_probability, 1.0f)){
    pixel_probability += 0.1;
    neighbour_probability -= 0.1;
  }
  else if (SAFE_EQUALS(pixel_probability, 1.0f) && SAFE_EQUALS(neighbour_probability, 0.0f)){
    pixel_probability -= 0.1;
    neighbour_probability += 0.1;
  }

  return (pixel_probability - neighbour_probability) / (((heaviside_value*pixel_probability) + ((1 - heaviside_value)*neighbour_probability)) + EPS);

}

float ComponentLevelSet::GetErrorValue(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_label, const size_t neighbour_label) const {

  const cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);

  float pixel_probability = re[target_label]; //
  float neighbour_probability = re[neighbour_label];

  const float heaviside_value = HeavisideFunction(sdf_value);

  if (SAFE_EQUALS(pixel_probability, 0.0f) && SAFE_EQUALS(neighbour_probability, 1.0f)){
    pixel_probability += 0.1;
    neighbour_probability -= 0.1;
  }
  else if (SAFE_EQUALS(pixel_probability, 1.0f) && SAFE_EQUALS(neighbour_probability, 0.0f)){
    pixel_probability -= 0.1;
    neighbour_probability += 0.1;
  }

  float v = (heaviside_value * pixel_probability) + ((1 - heaviside_value)*(neighbour_probability));
  v += 0.0000001f;
  return -log(v);

}

void ComponentLevelSet::ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &front_normal_image) {

  //find all the pixels which project to intersection points on the model
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);

  cv::Mat front_depth, back_depth;
  RenderModelForDepthAndContour(mesh, camera, front_depth, back_depth, front_normal_image);

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

void ComponentLevelSet::RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &front_normal_image){

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


  front_normal_image = ci::toOcv(front_depth_framebuffer_.getTexture(1));
  cv::flip(front_normal_image, front_normal_image, 0);

}