#include <cinder/Camera.h>
#include <cinder/gl/Light.h>
#include <CinderOpenCV.h>
#include <cinder/CinderMath.h>

#include "../../../include/ttrack/track/localizer/levelsets/comp_ls.hpp"
#include "../../../include/ttrack/resources.hpp"
#include "../../../include/ttrack/constants.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"

using namespace ttrk;

#define GRAD_DESCENT

#ifdef USE_CUDA
#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"
#endif 

ComponentLevelSet::ComponentLevelSet(size_t number_of_components, boost::shared_ptr<StereoCamera> camera) : StereoPWP3D(camera) {

  ci::gl::Fbo::Format format;
  format.setColorInternalFormat(GL_RGBA32F);
  format.enableColorBuffer(true, 1);
  component_map_framebuffer_ = ci::gl::Fbo(camera->left_eye()->Width(), camera->left_eye()->Height(), format);

  LoadShaders();

  for (size_t i = 0; i < number_of_components; ++i){
    components_.push_back(HomogenousComponent(i));
  }
  
}

ComponentLevelSet::~ComponentLevelSet(){

  component_map_framebuffer_.getDepthTexture().setDoNotDispose(false);
  component_map_framebuffer_.getTexture().setDoNotDispose(false);
  component_map_framebuffer_.reset();

}

void ComponentLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;

#ifdef USE_CERES

  current_model_ = current_model;
  TrackTargetCeresOptimization();

//#elif defined (USE_CUDA)
  
 
#else

  if (!current_model->cam) current_model->cam = stereo_camera_->left_eye();

  if (curr_step == 0) {
    
    best_region_score = std::numeric_limits<float>::min();
    region_scores.clear();

    cv::Mat front_intersection_image, back_intersection_image;
    ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), front_intersection_image, back_intersection_image);
    auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
    
    //if (first_run_){// || point_registration_->NeedsReset()){
    cv::Mat left_sdf_image, right_sdf_image, sdf_image(frame_->GetImage().rows, frame_->GetImage().cols, CV_32FC1);
    StereoPWP3D::ProcessSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, front_intersection_image, back_intersection_image);
    Localizer::ResetOcclusionImage();
    StereoPWP3D::ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, cv::Mat(), cv::Mat());


    if (current_model->NeedsModelRetrain()){
      current_model->RetrainModel(stereo_frame->GetLeftImage(), left_sdf_image, component_map_);
    }

    left_sdf_image.copyTo(sdf_image(cv::Rect(0, 0, left_sdf_image.cols, left_sdf_image.rows)));
    right_sdf_image.copyTo(sdf_image(cv::Rect(left_sdf_image.cols, 0, left_sdf_image.cols, left_sdf_image.rows)));
    current_model->ClassifyFrame(frame_, sdf_image);

    if (point_registration_ && !current_model->mps.is_initialised){
      point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);
      point_registration_->InitializeTracker(stereo_frame->GetLeftImage(), current_model);
    }
    else if (point_registration_ && current_model->mps.is_initialised){
      point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);
      //point_registration_->InitializeTracker(current_model->mps.previous_frame, current_model);
      point_registration_->TrackLocalPoints(stereo_frame->GetLeftImage(), current_model);
    }
      //boost::dynamic_pointer_cast<LKTracker3D>(point_registration_)->SetSpatialDerivatives(current_model->GetBasePose());
    //} 
    //else{

    //if (point_registration_){
    //point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);
    //boost::dynamic_pointer_cast<LKTracker3D>(point_registration_)->SetSpatialDerivatives(current_model->GetBasePose());  
    //  }

    //}
    
  }


  float error = DoAlignmentStep(current_model, true);
  UpdateWithErrorValue(error);
  errors_.push_back(error);

#endif

}

float ComponentLevelSet::DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  std::vector<cv::Matx<float, 7, 1> > region_jacobians;
  std::vector<cv::Matx<float, 7, 7> > region_hessian_approxs;
  std::vector<float> compls_region_scores;
  for (size_t i = 0; i < components_.size() - 1; ++i){
    region_jacobians.push_back(cv::Matx<float, 7, 1>::zeros());
    region_hessian_approxs.push_back(cv::Matx<float, 7, 7>::zeros());
    compls_region_scores.push_back(0.0f);
  }

  cv::Matx<float, 7, 1> points_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> points_hessian_approx = cv::Matx<float, 7, 7>::zeros();

  float current_score = 0.0f; //actual score
  float best_score = 0.0f; //best achieveable score given the contour
  ComputeScores(stereo_frame->GetLeftClassificationMap(), current_score, best_score);

  if (use_level_sets_){
    ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), region_jacobians, region_hessian_approxs, compls_region_scores);
    //ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), region_jacobians, region_hessian_approxs, compls_region_scores);
  }

  if (track_points && point_registration_)
    ComputeLKJacobian(current_model, points_jacobian, points_hessian_approx);

  //cv::Mat front_intersection_image, back_intersection_image;
  //ProcessSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), front_intersection_image, back_intersection_image);
  //point_registration_->SetFrontIntersectionImage(front_intersection_image);
  //boost::dynamic_pointer_cast<LKTracker3D>(point_registration_)->TrackLocalPoints(stereo_frame->GetLeftImage(), current_model->GetBasePose());
  //std::vector<float> point_jacs = boost::dynamic_pointer_cast<LKTracker3D>(point_registration_)->GetUpdateStep();
#ifdef GRAD_DESCENT

  cv::Matx<float, 7, 1> region_jacobian = cv::Matx<float, 7, 1>::zeros();
  for (auto jac : region_jacobians){
    region_jacobian = region_jacobian + jac;
  }

  if (best_score > 0)
    region_scores.push_back(current_score / best_score);
    

  region_jacobian = region_jacobian + (point_registration_weight * points_jacobian);

  bool small_steps = false;
  if (curr_step > (0.5 * NUM_STEPS))
    small_steps = true;
  
  std::vector<float> region_jacs = ScaleRigidJacobian(region_jacobian, small_steps);
  //std::vector<float> point_jacs = ScaleRigidJacobian(points_jacobian);


  for(size_t i = 0; i < region_jacs.size(); ++i){

    //region_jacs[i] = (region_jacs[i] + (2 * point_jacs[i])) / 3;
    //region_jacs[i] = point_jacs[i];
  }


  current_model->UpdatePose(region_jacs);

#else

  auto identity = region_hessian_approx * region_hessian_approx.inv();
  ci::app::console() << "Identity = " << identity << std::endl;

  region_jacobian = region_hessian_approx.inv() * region_jacobian;
  points_jacobian = points_hessian_approx.inv() * points_jacobian;
  std::vector<float> jacs(7, 0);
  for (size_t v = 0; v < 7; ++v){
    //jacs[v] = -(region_jacobian(v) + (0*points_jacobian(v)))/2.0f;
    jacs[v] = -region_jacobian(v);
  }
  current_model->UpdatePose(jacs);

#endif

  if (track_points && point_registration_)
    point_registration_->UpdatePointsOnModelAfterDerivatives(current_model, current_model->GetBasePose());
  
  return error;

}

void ComponentLevelSet::LoadShaders(){

  component_shader_ = ci::gl::GlslProg(ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_VERT), ci::app::loadResource(COMP_LS_BACK_DEPTH_AND_CONTOUR_FRAG));

}

void ComponentLevelSet::ComputeScores(const cv::Mat &classification_image, float &current_score, float &best_score) const {

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){


      bool cont = true;
      for (size_t comp = 1; comp < components_.size(); ++comp){
        if (components_[comp].sdf_image.at<float>(r, c))
          if (std::abs(components_[comp].sdf_image.at<float>(r, c)) < 6){
            cont = false;
            break;
          }
      }
      if (cont) continue; //only count pixels 'near' the contour.

      size_t target_label = component_map_.at<unsigned char>(r, c);
      size_t neighbour_label = 0;
      float min_negative_distance = std::numeric_limits<float>::max();
      //if background then check
      if (target_label == 0){
        for (size_t comp = 1; comp < components_.size(); ++comp){
          const float sdf_val = std::abs(components_[comp].sdf_image.at<float>(r, c));
          if (sdf_val < min_negative_distance){
            neighbour_label = comp;
            min_negative_distance = sdf_val;
          }
        }
      }
      else{

        const float distance_to_edge = components_[target_label].sdf_image.at<float>(r, c);
        const float distance_from_neighbour = 0;
        min_negative_distance = std::numeric_limits<float>::max();
        size_t best_component_neighbour = 0;
        for (size_t comp = 1; comp < components_.size(); ++comp){
          if (comp == target_label) continue;
          const float sdf_val = std::abs(components_[comp].sdf_image.at<float>(r, c));
          if (sdf_val < min_negative_distance){
            min_negative_distance = sdf_val;
            best_component_neighbour = comp;
          }
        }

        if (min_negative_distance < distance_to_edge){
          neighbour_label = best_component_neighbour;
        }
        else{
          neighbour_label = 0;
        }

      }

      const cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);
      float pixel_probability = re[target_label]; //
      float neighbour_probability = re[neighbour_label];


      float sdf_value = 0;
      if (target_label == 0)
        sdf_value = components_[neighbour_label].sdf_image.at<float>(r, c);
      else
        sdf_value = components_[target_label].sdf_image.at<float>(r, c);


      current_score += (pixel_probability * HeavisideFunction(sdf_value) + ((1 - HeavisideFunction(sdf_value)) * neighbour_probability));

      if (target_label == 0){
        best_score += (1 - HeavisideFunction(sdf_value));
      }
      else{
        best_score += (HeavisideFunction(sdf_value));
      }
    }
  }

}

void ComponentLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, std::vector<cv::Matx<float, 7, 1> > &jacobians, std::vector<cv::Matx<float, 7, 7> > &hessian_approxs, std::vector<float> &error){
  
  cv::Mat front_intersection_image, back_intersection_image;

  ProcessSDFAndIntersectionImage(current_model, camera, front_intersection_image, back_intersection_image);

  for (size_t comp = 1; comp < components_.size(); ++comp){

    cv::Mat &sdf_image = components_[comp].sdf_image;

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
        
           if (occlusion_image.at<float>(r, c) < (front_intersection_data[(i * 3) + 2] - 0.1)) {
             continue;
           }

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
                    
          float region_agreement = 0;

          if (target_label == 0 || nearest_different_neighbour_label == 0){
            region_agreement = GetBinaryRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);
          }
          else{
            region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);
          }

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
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
          else{
            ci::app::console() << "Error, this is an invalid camera!!!" << std::endl;
            throw std::runtime_error("");
          }

          jacobians[comp-1] += jacs.t();
          hessian_approxs[comp-1] += (jacs.t() * jacs);

        }
      }
    }
  }

}

unsigned char ComponentLevelSet::ComputeNearestNeighbourForPixel(const int r, const int c, const float sdf_value, const int width, const int height) const{

  const cv::Mat &component_map_m = component_map_;

  const unsigned char pixel_class = component_map_.at<unsigned char>(r, c);

  const unsigned char *comp_map_data = component_map_.data;

  const int ceil_sdf = (int)std::abs(ceil(sdf_value)) + 1;

  for (int w_c = c - ceil_sdf; w_c <= c + ceil_sdf; ++w_c){

    const int up_idx = (r + ceil_sdf)*width + w_c;
    const int down_idx = (r - ceil_sdf)*width + w_c;
    if ((up_idx >= 0 && up_idx < (height*width)) && comp_map_data[up_idx] != pixel_class){
      return comp_map_data[up_idx];
    }
    else if ((down_idx >= 0 && down_idx < (height*width)) && comp_map_data[down_idx] != pixel_class){
      return comp_map_data[down_idx];
    }
  }

  for (int w_r = r - ceil_sdf; w_r <= r + ceil_sdf; ++w_r){

    const int left_idx = w_r*width + c - ceil_sdf;
    const int right_idx = w_r*width + c + ceil_sdf;
    if ((left_idx >= 0 && left_idx < (height*width)) && comp_map_data[left_idx] != pixel_class){
      return comp_map_data[left_idx];
    }
    else if ((right_idx >= 0 && right_idx < (height*width)) && comp_map_data[right_idx] != pixel_class){
      return comp_map_data[right_idx];
    }

  }
  
  return 255;

}

void ComponentLevelSet::GetRegionProbability(const cv::Mat &classification_image, const int r, const int c, const size_t target_label, const size_t neighbour_label, float &pixel_probability, float &neighbour_probability) const{

  cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);

  pixel_probability = re[target_label]; 
  neighbour_probability = re[neighbour_label];

}

float ComponentLevelSet::GetBinaryRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_label, const size_t neighbour_label) const {

  float pixel_probability;
  float neighbour_probability;
  cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);

  if (target_label == 0){
    pixel_probability = re[0];
    neighbour_probability = re[1] + re[2] + re[3] + re[4];
  }
  else{
    neighbour_probability = re[0];
    pixel_probability = re[1] + re[2] + re[3] + re[4];
  }
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


float ComponentLevelSet::GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_label, const size_t neighbour_label) const {

  float pixel_probability;
  float neighbour_probability;
 
  GetRegionProbability(classification_image, r, c, target_label, neighbour_label, pixel_probability, neighbour_probability);
  
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

float ComponentLevelSet::GetErrorValue(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_label, const size_t neighbour_label, const size_t foreground_size, const size_t background_size) const {

  const cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);

  float pixel_probability = re[target_label]; //
  float neighbour_probability = re[neighbour_label];

  const float heaviside_value = HeavisideFunction(sdf_value);

  if (SAFE_EQUALS(pixel_probability, 0.0f) && SAFE_EQUALS(neighbour_probability, 1.0f)){
    pixel_probability += 0.01;
    neighbour_probability -= 0.01;
  }
  else if (SAFE_EQUALS(pixel_probability, 1.0f) && SAFE_EQUALS(neighbour_probability, 0.0f)){
    pixel_probability -= 0.01;
    neighbour_probability += 0.01;
  }

  float v = ((heaviside_value/foreground_size) * pixel_probability) + (((1 - heaviside_value)/background_size)*(neighbour_probability));
  v += 0.0000001f;

  if (v < 0){
    int axa = 0;
  }

  float nv = log(v);
  float nnv = -nv;
  return nnv;

}

void ComponentLevelSet::ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image) {

  //find all the pixels which project to intersection points on the model
  front_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);
  back_intersection_image = cv::Mat::zeros(frame_->GetImageROI().size(), CV_32FC3);

  cv::Mat front_depth, back_depth;
  RenderModelForDepthAndContour(mesh, camera, front_depth, back_depth);

  Localizer::UpdateOcclusionImage(front_depth);

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

  for (size_t i = 1; i < components_.size(); i++){
    components_[i].sdf_image = cv::Mat(front_depth.size(), CV_32FC1);

#ifdef USE_CUDA
    ttrk::gpu::distanceTransform(components_[i].contour_image, components_[i].sdf_image);
#else
    distanceTransform(~components_[i].contour_image, components_[i].sdf_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);
#endif    

  }

  for (int r = 0; r < front_depth.rows; ++r){
    for (int c = 0; c < front_depth.cols; ++c){
      for (size_t i = 1; i < components_.size(); i++){
        if (components_[i].target_probability != component_map_.at<unsigned char>(r, c)){
          components_[i].sdf_image.at<float>(r, c) *= -1;
        }
      }
    }
  }

  //just do the first one
  if (components_.size() > 0)
    progress_frame_ = ComputePrettySDFImage(components_[1].sdf_image);

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
  cv::flip(front_depth_flipped, front_depth, 0);
  front_depth_flipped.release();

  cv::Mat back_depth_flipped = ci::toOcv(back_depth_framebuffer_.getTexture(0));
  cv::flip(back_depth_flipped, back_depth, 0);
  back_depth_flipped.release();

  cv::Mat f_component_map = ci::toOcv(component_map_framebuffer_.getTexture());
  cv::flip(f_component_map, f_component_map, 0);

  float *comp_src = (float *)f_component_map.data;
  component_map_ = cv::Mat(f_component_map.size(), CV_8UC1);
  unsigned char *comp_dst = (unsigned char *)component_map_.data;

  //get the binary component images and the component map.
  for (size_t i = 0; i < components_.size(); ++i){
    components_[i].binary_image = cv::Mat::zeros(front_depth.size(), CV_8UC1);
    components_[i].contour_image = cv::Mat::zeros(front_depth.size(), CV_8UC1);
  }


  for (int r = 0; r < front_depth.rows; ++r){
    for (int c = 0; c < front_depth.cols; ++c){
      if (std::abs(comp_src[(r * front_depth.cols + c) * 4]) < EPS){
        comp_dst[r * front_depth.cols + c] = 1; //plastic shaft
        components_[1].binary_image.at<unsigned char>(r, c) = 255;
      }
      else if (components_.size() > 2 && std::abs(comp_src[(r * front_depth.cols + c) * 4] - 0.686) < 0.01){
        comp_dst[r * front_depth.cols + c] = 2; //metal head
        components_[2].binary_image.at<unsigned char>(r, c) = 255;
      }
      else{
        comp_dst[r * front_depth.cols + c] = 0; //background
        components_[0].binary_image.at<unsigned char>(r, c) = 255;
      }
    }
  }

  //create the contour map from the 

  for (int r = 1; r < front_depth.rows - 1; ++r){
    for (int c = 1; c < front_depth.cols - 1; ++c){
      for (int i = 0; i < components_.size(); ++i){
        if (IsGreaterThanNeighbour(components_[i].binary_image, r, c)){
          components_[i].contour_image.at<unsigned char>(r, c) = 255;
        }
      }
    }
  }


  //front_normal_image = ci::toOcv(front_depth_framebuffer_.getTexture(1));
  //cv::flip(front_normal_image, front_normal_image, 0);

}

#ifdef USE_CERES

struct CeresCostFunction : public ceres::SizedCostFunction<401860, 3, 4> {

public:

  CeresCostFunction(ComponentLevelSet &pwp3d) : pwp3d_(pwp3d) {}
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

    return pwp3d_(parameters, residuals, jacobians);

  }

  ComponentLevelSet &pwp3d_;


};

void ComponentLevelSet::DoEyeCeres(double const *const *parameters, double *residuals, double **jacobians, const cv::Mat &classification_image, const boost::shared_ptr<MonocularCamera> camera) const {

  const ci::Vec3f translation(parameters[0][0], parameters[0][1], parameters[0][2]);
  ci::Quatf rotation(parameters[1][0], parameters[1][1], parameters[1][2], parameters[1][3]);

  Pose p(rotation, translation);
  current_model_->SetBasePose(p);

  cv::Mat front_intersection_image, back_intersection_image;
  const_cast<ComponentLevelSet *>(this)->ProcessSDFAndIntersectionImage(current_model_, stereo_camera_->left_eye(), front_intersection_image, back_intersection_image);

  float residual_sum = 0;

  for (size_t comp = 1; comp < components_.size(); ++comp){

    const cv::Mat &sdf_image = components_[comp].sdf_image;

    cv::Mat target_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat nearest_neighbour_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat region_agreement_im = cv::Mat::zeros(sdf_image.size(), CV_32FC1);

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;

    size_t ceres_matrix_index = 0;

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c, ++ceres_matrix_index){

        const int i = r*classification_image.cols + c;

        assert(sdf_image.at<float>(r, c) == sdf_im_data[i]);

        size_t target_label = components_[comp].target_probability;

        //find the nearest neighbouring pixel with a different label to this one - if we are inside the contour then search for the nearest different label, if we are outside the contour (i.e. looking at 'background') then just choose the pixel.
        size_t nearest_different_neighbour_label;
        if (sdf_im_data[i] >= 0){
          nearest_different_neighbour_label = ComputeNearestNeighbourForPixel(r, c, sdf_im_data[i], classification_image.cols, classification_image.rows);
        }
        else{
          nearest_different_neighbour_label = component_map_.at<unsigned char>(r, c);
        }

        if (nearest_different_neighbour_label > 2){
          if(target_label != 0) nearest_different_neighbour_label = 0;
          else nearest_different_neighbour_label = 1;
        }

        float Pf;
        float Pb;
        GetRegionProbability(classification_image, r, c, target_label, nearest_different_neighbour_label, Pf, Pb);

        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){
          residuals[ceres_matrix_index] = -log((double)(HeavisideFunction(sdf_im_data[i])*Pf + (1 - HeavisideFunction(sdf_im_data[i]))*Pb) + 0.000001);
        }
        else{
          if (sdf_im_data[i] > 0){
            residuals[ceres_matrix_index] = -log(Pf + 0.000001);
          }
          else{
            residuals[ceres_matrix_index] = -log(Pb + 0.000001);
          }
        }


        //if we are computing the jacobians on this loop
        if (jacobians != NULL && (jacobians[0] != NULL || jacobians[1] != NULL)){
          //if inside contour
          if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

            int new_i = i;
            int new_r = r;
            int new_c = c;

            if (sdf_im_data[i] < 0.0) {
              int closest_r, closest_c;
              bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
              if (!found) throw std::runtime_error("");
              new_i = closest_r * sdf_image.cols + closest_c;
              new_r = closest_r;
              new_c = closest_c;
            }


            cv::Matx<float, 1, 7> jacs = cv::Matx<float, 1, 7>::zeros();

            //warning - do not use shifted index points for sdf.
            const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
            const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

            const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);

            //update the jacobian
            if (camera == stereo_camera_->left_eye())
              const_cast<ComponentLevelSet *>(this)->UpdateJacobian(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(new_i), back_intersection_image.at<cv::Vec3f>(new_i), current_model_, jacs);
            else
              const_cast<ComponentLevelSet *>(this)->UpdateJacobianRightEye(region_agreement, sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->right_eye()->Fx(), stereo_camera_->right_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(new_i), back_intersection_image.at<cv::Vec3f>(new_i), current_model_, jacs);

            for (int j_idx = 0; j_idx < 3; ++j_idx) jacobians[0][ceres_matrix_index * 3 + j_idx] = jacs(j_idx);
            for (int j_idx = 0; j_idx < 4; ++j_idx) jacobians[1][ceres_matrix_index * 4 + j_idx] = jacs(3 + j_idx);


          }
          else{

            if (jacobians[0] != NULL){ for (int j_idx = 0; j_idx < 3; ++j_idx) jacobians[0][ceres_matrix_index * 3 + j_idx] = 0; }
            if (jacobians[1] != NULL){ for (int j_idx = 0; j_idx < 4; ++j_idx) jacobians[1][ceres_matrix_index * 4 + j_idx] = 0; }

          }
        }
      }
    }
  }
}

bool ComponentLevelSet::operator() (double const *const *parameters, double *residuals, double **jacobians) const{

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  DoEyeCeres(parameters, residuals, jacobians, stereo_frame->GetLeftClassificationMap(), stereo_camera_->left_eye());
  //DoEyeCeres(parameters, residuals, jacobians, stereo_frame->GetRightClassificationMap(), stereo_camera_->right_eye());

  return true;

}

bool ComponentLevelSet::operator() (double const* const* parameters, double* residuals) const{

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  DoEyeCeres(parameters, residuals, nullptr, stereo_frame->GetLeftClassificationMap(), stereo_camera_->left_eye());
  //DoEyeCeres(parameters, residuals, jacobians, stereo_frame->GetRightClassificationMap(), stereo_camera_->right_eye());

  return true;


}

void ComponentLevelSet::TrackTargetCeresOptimization(){

  std::vector<float> pose_parameters;
  current_model_->GetPose(pose_parameters);

  std::vector<double*> parameter_blocks;
  
  double *parameters_t = new double[3];
  parameter_blocks.push_back(parameters_t);
  double *parameters_r = new double[4];
  parameter_blocks.push_back(parameters_r);


  /*
  static bool init_run = true;
  if (init_run){
    parameters[0][0] = pose_parameters[0] + 0.225;
    parameters[0][1] = pose_parameters[1] + 1.825;
    parameters[0][2] = pose_parameters[2] + 0.125;
    init_run = false;
  }
  else{
  */
  parameter_blocks[0][0] = pose_parameters[0];
  parameter_blocks[0][1] = pose_parameters[1];
  parameter_blocks[0][2] = pose_parameters[2];
  //}
  parameter_blocks[1][0] = pose_parameters[3]; //w
  parameter_blocks[1][1] = pose_parameters[4]; //x
  parameter_blocks[1][2] = pose_parameters[5]; //y
  parameter_blocks[1][3] = pose_parameters[6]; //z

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  ceres::CostFunction *cost_function = new CeresCostFunction(*this);
  //ceres::DynamicNumericDiffCostFunction<StereoPWP3D> *cost_function = new ceres::DynamicNumericDiffCostFunction<StereoPWP3D>(this, ceres::Ownership::DO_NOT_TAKE_OWNERSHIP, 0.0001);
  //cost_function->AddParameterBlock(3);
  //cost_function->AddParameterBlock(4);
  //cost_function->SetNumResiduals(left_image.rows*left_image.cols);




  ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, NULL, parameter_blocks);
  ceres::QuaternionParameterization *qp = new ceres::QuaternionParameterization();

  problem.SetParameterization(parameter_blocks[1], qp);

  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 15;
  //minimizer can be line search or trust region
  //line search - choose size and then find direction that minimizes the function
  //trust region - choose a direction 
  //options.minimizer_type = ceres::LINE_SEARCH;
  //options.line_search_direction_type = ceres::STEEPEST_DESCENT;
  //options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  ci::app::console() << "Done" << summary.FullReport() << "\n" << std::endl;

  const ci::Vec3f translation(parameter_blocks[0][0], parameter_blocks[0][1], parameter_blocks[0][2]);
  ci::Quatf rotation(parameter_blocks[1][0], parameter_blocks[1][1], parameter_blocks[1][2], parameter_blocks[1][3]);
  
  current_model_->SetBasePose(Pose(rotation, translation));

  UpdateWithErrorValue(10);

  delete cost_function;
  delete qp;
  delete[] parameter_blocks[0];
  delete[] parameter_blocks[1];


}




#endif