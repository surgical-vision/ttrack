#include <cinder/app/App.h>
#include <numeric>

#include "../../../include/ttrack/track/localizer/levelsets/articulated_level_set.hpp"
#include "../../../include/ttrack/constants.hpp"

#ifdef USE_CUDA
#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"
#endif 

using namespace ttrk;

ArticulatedComponentLevelSet::ArticulatedComponentLevelSet(size_t number_of_articulated_components, size_t number_of_level_set_components, boost::shared_ptr<StereoCamera> camera) : ComponentLevelSet(number_of_level_set_components, camera), number_of_articulated_components_(number_of_articulated_components), current_pose_param_(0), previous_error_value_(-1.0f), optimization_type_(GRADIENT_DESCENT){ }


void ArticulatedComponentLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;

  if (!current_model->cam) current_model->cam = stereo_camera_->left_eye();

  if (curr_step == 0) {

    cv::Mat left_sdf_image, right_sdf_image, sdf_image(frame_->GetImage().rows, frame_->GetImage().cols, CV_32FC1), left_frame_idx_image, right_frame_idx_image;
    cv::Mat front_intersection_image, back_intersection_image;
    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, front_intersection_image, back_intersection_image, right_frame_idx_image);
    cv::Mat right_component_image = component_map_.clone();
    Localizer::ResetOcclusionImage();
    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, front_intersection_image, back_intersection_image, left_frame_idx_image);

    auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

    if (current_model->NeedsModelRetrain()){

      cv::Mat whole_image = stereo_frame->GetImage();
      cv::Mat whole_sdf_image(left_sdf_image.rows, left_sdf_image.cols * 2, CV_32FC1);
      cv::Mat whole_component_image(left_frame_idx_image.rows, left_frame_idx_image.cols * 2, CV_8UC1);
      left_sdf_image.copyTo(whole_sdf_image(cv::Rect(0, 0, left_sdf_image.cols, left_sdf_image.rows)));
      right_sdf_image.copyTo(whole_sdf_image(cv::Rect(left_sdf_image.cols, 0, left_sdf_image.cols, left_sdf_image.rows)));
      component_map_.copyTo(whole_component_image(cv::Rect(0, 0, left_sdf_image.cols, left_sdf_image.rows)));
      right_component_image.copyTo(whole_component_image(cv::Rect(left_sdf_image.cols, 0, left_sdf_image.cols, left_sdf_image.rows)));

      current_model->RetrainModel(whole_image, whole_sdf_image, whole_component_image);

      //current_model->RetrainModel(stereo_frame->GetLeftImage(), left_sdf_image, component_map_);
    }

    //static bool first = true;
    //if (first){
    //  NUM_STEPS = 500;
    //  std::vector<float> updates;
    //  for (int j = 0; j < 11; ++j) updates.push_back(0.0f);
    //  updates[8] += 0.8;
    //  updates[9] += 0.4;
    //  updates[10] += 0.4;

    //  current_model->UpdatePose(updates);
    //  first = false;

    //  ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, front_intersection_image, back_intersection_image, right_frame_idx_image);
    //  right_component_image = component_map_.clone();
    //  Localizer::ResetOcclusionImage();
    //  ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, front_intersection_image, back_intersection_image, left_frame_idx_image);

    //}

    left_sdf_image.copyTo(sdf_image(cv::Rect(0, 0, left_sdf_image.cols, left_sdf_image.rows)));
    right_sdf_image.copyTo(sdf_image(cv::Rect(left_sdf_image.cols, 0, left_sdf_image.cols, left_sdf_image.rows)));
    current_model->ClassifyFrame(frame_, sdf_image);


    if (frame_count_ % 8 == 0) current_model->mps.is_initialised = false;

    point_registration_->SetFrontIntersectionImage(front_intersection_image, current_model);

    //if (frame_count_ != 0){

    if (point_registration_ && !current_model->mps.is_initialised){

      point_registration_->InitializeTracker(stereo_frame->GetLeftImage(), current_model, left_frame_idx_image);

    }
    else if (point_registration_ && current_model->mps.is_initialised){

      if (boost::dynamic_pointer_cast<ArticulalatedLKTrackerFrameToFrame>(point_registration_)){
        boost::dynamic_pointer_cast<ArticulalatedLKTrackerFrameToFrame>(point_registration_)->TrackLocalPoints(stereo_frame->GetLeftImage(), current_model, left_frame_idx_image);
      }
      else{
        point_registration_->TrackLocalPoints(stereo_frame->GetLeftImage(), current_model);
      }

    }
    //}




  }

  double t = cv::getTickCount();

  float error = DoAlignmentStep(current_model, true && current_model->mps.is_initialised);

  ci::app::console() << "Time taken = " << (cv::getTickCount() - t) / cv::getTickFrequency() << std::endl;

  UpdateWithErrorValue(error);
  errors_.push_back(error);

}


cv::Vec2f ArticulatedComponentLevelSet::CheckCloseClasper(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera) {

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float update_step_for_joint[2];
  update_step_for_joint[0] = 0.1;
  update_step_for_joint[1] = 0.1;

  cv::Vec2f jacobian(0, 0);

  std::vector<float> backup_pose;
  //postive update for head
  current_model->GetPose(backup_pose);

  for (int i = 0; i < 2; ++i){

    std::vector<float> updates;
    for (int j = 0; j < 11; ++j) updates.push_back(0.0f);

    const size_t idx = 9 + i;

    //not used atm
    std::vector<int> articulated_components;

    //if the clasper angle is greater than a threshold

  }

  //jacobian[3] = jacobian[2];
  ci::app::console() << "Jacobian = " << jacobian << std::endl;

  return jacobian;

}


cv::Vec4f ArticulatedComponentLevelSet::ComputeFDJacobianForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera){

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float update_step_for_joint[4];
  update_step_for_joint[0] = 0.08;
  update_step_for_joint[1] = 0.05;
  update_step_for_joint[2] = 0.1;
  update_step_for_joint[3] = 0.1;

  cv::Vec4f jacobian(0, 0, 0, 0);

  std::vector<float> backup_pose;
  //postive update for head
  current_model->GetPose(backup_pose);

  for (int i = 0; i < 4; ++i){

    std::vector<float> updates;
    for (int j = 0; j < 11; ++j) updates.push_back(0.0f);

    if (i == 1) continue;

    const size_t idx = 7 + i;

    //not used atm
    std::vector<int> articulated_components;

    //check positive step
    float e1 = 0;
    //if (i >= 1){
    // e1 = std::numeric_limits<float>::max();
    //}
    //else{
    updates[idx] += update_step_for_joint[i];
    current_model->UpdatePose(updates);
    e1 = GetErrorForPose(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), articulated_components);
    current_model->SetPose(backup_pose);
    //}

    //check zero step
    updates[idx] -= update_step_for_joint[i];
    current_model->UpdatePose(updates);
    float e2 = GetErrorForPose(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), articulated_components);
    current_model->SetPose(backup_pose);

    float e3 = 0;
    //if (i >= 1){ only check if at limit
    //  e3 = std::numeric_limits<float>::max();
    //}
    //else{
    //check negative step
    updates[idx] -= update_step_for_joint[i];
    current_model->UpdatePose(updates);
    e3 = GetErrorForPose(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), articulated_components);
    current_model->SetPose(backup_pose);
    //}

    if (e1 < e2 && e1 < e3) jacobian[i] = update_step_for_joint[i];
    else if (e3 < e2 && e3 < e1) jacobian[i] = -update_step_for_joint[i]; //because we minus it in the update step
    else jacobian[i] = 0;
  }

  //jacobian[3] = jacobian[2];
  ci::app::console() << "Jacobian = " << jacobian << std::endl;

  return jacobian;

}

void ArticulatedComponentLevelSet::ComputeArticulatedAreas(const cv::Mat &sdf, size_t &fg_area, size_t &bg_area, std::vector<int> articulated_indexes, const cv::Mat &index_image) const {

  fg_area = bg_area = 0;

  for (auto r = 0; r < sdf.rows; ++r){
    for (auto c = 0; c < sdf.rows; ++c){

      if (sdf.at<float>(r, c) <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf.at<float>(r, c) >= -float(HEAVYSIDE_WIDTH) + 1e-1){


        int articulated_component = index_image.at<unsigned char>(r, c);
        if (articulated_component == 255){
          articulated_component = ComputeNearestArticulatedComponent(r, c, sdf.at<float>(r, c), sdf.cols, sdf.rows, index_image);
        }

        bool pixel_is_articulated_component = false;
        for (auto a_comp : articulated_indexes) if (articulated_component == a_comp) pixel_is_articulated_component = true;
        if (!pixel_is_articulated_component) continue;


        const float sdf_v = sdf.at<float>(r, c);

        if (sdf_v > 0){
          fg_area++;
        }
        else{
          bg_area++;
        }

      }

    }
  }

}

float ArticulatedComponentLevelSet::GetErrorForPose(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, const std::vector<int> &joint_indexes){

  cv::Mat composite_sdf_image, front_intersection_image, back_intersection_image, index_image;

  //Use the articulated components to compute the index_image which is an image where each pixel indexes the articulated component used in the optimization
  ProcessArticulatedSDFAndIntersectionImage(current_model, camera, composite_sdf_image, front_intersection_image, back_intersection_image, index_image);

  size_t foreground_size = 0;
  size_t background_size = 0;
  ComputeArticulatedAreas(composite_sdf_image, foreground_size, background_size, joint_indexes, index_image);

  cv::Mat sdf_image_1 = components_[0].sdf_image;
  cv::Mat sdf_image_2 = components_[1].sdf_image;
  cv::Mat sdf_image_3 = components_[2].sdf_image;

  float error = 0;
  size_t num_pixels = 0;
  //iterate over the multiply component level set (plastic and metal)
  for (size_t comp = 1; comp < components_.size(); ++comp){

    cv::Mat &sdf_image = components_[comp].sdf_image;

    cv::Mat target_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat nearest_neighbour_label_image = cv::Mat::zeros(sdf_image.size(), CV_8UC1);
    cv::Mat region_agreement_im = cv::Mat::zeros(sdf_image.size(), CV_32FC1);

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;

    cv::Mat error_image = cv::Mat::zeros(front_intersection_image.size(), CV_8UC1);

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c){

        const int i = r*classification_image.cols + c;

        assert(sdf_image.at<float>(r, c) == sdf_im_data[i]);

        size_t target_label = components_[comp].target_probability;
        size_t nearest_different_neighbour_label = 0;
        if (target_label == 0)
          nearest_different_neighbour_label = 1; //FIX THIS

        if (cv::sum(classification_image.at<cv::Vec<float, 5> >(r, c)) == cv::Scalar(0)){
          continue;
        }

        float sdf_value = sdf_im_data[i];
        if (sdf_im_data[i] > 1) sdf_value = 1.0;
        else if (sdf_im_data[i] < -1) sdf_value = -1.0;
        float pix_err = GetErrorValue(classification_image, r, c, sdf_value, target_label, nearest_different_neighbour_label, index_image.at<unsigned char>(r, c), 1, 1);

        error += pix_err;
        num_pixels++;

      }
    }
  }

  if (use_articulated_point_derivs_)
    error += (point_registration_weight * GetPointProjectionError(current_model, camera, index_image));

  if (num_pixels > 0) return error / num_pixels;
  else return error;
}

float ArticulatedComponentLevelSet::GetPointProjectionError(boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, const cv::Mat &articulated_index_image){

  if (!point_registration_) return 0.0f;

  return point_registration_->GetPointProjectionError(current_model, camera, articulated_index_image);
}

int ArticulatedComponentLevelSet::ComputeNearestArticulatedComponent(const int r, const int c, const float sdf_value, const int width, const int height, const cv::Mat &index_image) const{

  const unsigned char articulated_component = index_image.at<unsigned char>(r, c); //this should be background
  if (articulated_component != 255) {
    ci::app::console() << "Articulated component is not 255 at (" << r << ", " << c << ") -> " << articulated_component << std::endl;
    throw std::runtime_error("");
  }


  const unsigned char *index_image_data = index_image.data;

  const int ceil_sdf = (int)std::abs(ceil(sdf_value)) + 1;

  for (int w_c = c - ceil_sdf; w_c <= c + ceil_sdf; ++w_c){

    const int up_idx = (r + ceil_sdf)*width + w_c;
    const int down_idx = (r - ceil_sdf)*width + w_c;
    if ((up_idx >= 0 && up_idx < (height*width)) && index_image_data[up_idx] != articulated_component){
      return index_image_data[up_idx];
    }
    else if ((down_idx >= 0 && down_idx < (height*width)) && index_image_data[down_idx] != articulated_component){
      return index_image_data[down_idx];
    }
  }

  for (int w_r = r - ceil_sdf; w_r <= r + ceil_sdf; ++w_r){

    const int left_idx = w_r*width + c - ceil_sdf;
    const int right_idx = w_r*width + c + ceil_sdf;
    if ((left_idx >= 0 && left_idx < (height*width)) && index_image_data[left_idx] != articulated_component){
      return index_image_data[left_idx];
    }
    else if ((right_idx >= 0 && right_idx < (height*width)) && index_image_data[right_idx] != articulated_component){
      return index_image_data[right_idx];
    }

  }

  return 255;

}

void ArticulatedComponentLevelSet::ComputeArticulatedPointRegistrationJacobian(boost::shared_ptr<Model> current_model, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 4, 1> &articulated_jacobian, const cv::Mat &index_image){

  std::vector<float> rigid_derivs = point_registration_->GetDerivativesForPointsOnRigidBody(current_model, current_model->GetBasePose(), index_image);
  std::vector<float> articulated_derivs = point_registration_->GetArticulatedDerivativesForPoints(current_model, index_image);

  cv::Matx<float, 1, 7> rigid_jacobian_vals = cv::Matx<float, 1, 7>::zeros();

  for (int i = 0; i < rigid_derivs.size(); ++i){
    //rigid_jacobian_vals(i) += (int)point_registration_weight * rigid_derivs[i];

    if ((i < 3 && use_point_derivs_translation_) || (i >= 3 && use_point_derivs_rotation_))
      rigid_jacobian_vals(i) += articulated_derivs[i];
  }

  ci::app::console() << "Rigid point jacobian = " << rigid_jacobian_vals << std::endl;

  rigid_jacobian += rigid_jacobian_vals.t();

  cv::Matx<float, 1, 4> articulated_jacobian_vals = cv::Matx<float, 1, 4>::zeros();

  if (use_articulated_point_derivs_){
    for (int i = rigid_derivs.size(), j = 0; i < articulated_derivs.size(); ++i, j++){
      //articulated_jacobian_vals(j) += (int)point_registration_weight * articulated_derivs[i];
      articulated_jacobian_vals(j) += articulated_derivs[i];
    }
  }

  ci::app::console() << "Articulated point jacobian = " << articulated_jacobian_vals << std::endl;

  articulated_jacobian += articulated_jacobian_vals.t();

}

void ArticulatedComponentLevelSet::ApplyClasperLimitStateToJacobian(const boost::shared_ptr<Model> current_model, std::vector<float> &jacs, cv::Matx<float, 4, 1> region_articulated_jacobian, cv::Matx<float, 4, 1> point_articulated_jacobian, bool use_vals_from_sampling) {

  std::vector<float> backup_pose;
  //postive update for head
  current_model->GetPose(backup_pose);

  const float max_head_angle = 0.95;
  const float min_clasper_angle = -0.49;
  const float max_clasper_angle = 1.4;
  const float max_pointer_angle = 1.0;
  const float min_pointer_angle = -1.0;
  bool joint_closed = false;
  bool joint_opened = false;
  bool pointer_at_limit1 = false;
  bool pointer_at_limit2 = false;
  bool head_at_limit = false;
  if ((backup_pose[9] + backup_pose[10]) < min_clasper_angle) joint_closed = true;
  if ((backup_pose[9] + backup_pose[10]) > max_clasper_angle) joint_opened = true;
  if (backup_pose[8] > max_pointer_angle) pointer_at_limit1 = true;
  if (backup_pose[8] < min_pointer_angle) pointer_at_limit2 = true;

  if (std::abs(backup_pose[7]) >= max_head_angle) head_at_limit = true;

  ci::app::console() << "BP[7] = " << backup_pose[7] << "\n";
  ci::app::console() << "BP[8] = " << backup_pose[8] << "\n";
  ci::app::console() << "BP[9] = " << backup_pose[9] << "\n";
  ci::app::console() << "BP[10] = " << backup_pose[10] << "\n";

  if (joint_closed) ci::app::console() << "Joint closed\n";
  if (joint_opened) ci::app::console() << "Joint Opened\n";
  if (pointer_at_limit1) ci::app::console() << "Pointer at limit 1\n";
  if (pointer_at_limit2) ci::app::console() << "Pointer at limit 2" << std::endl;


  if (use_vals_from_sampling){
    jacs[7] = region_articulated_jacobian(0);
    jacs[8] = region_articulated_jacobian(1);
    jacs[9] = region_articulated_jacobian(2);
    jacs[10] = region_articulated_jacobian(3);
  }
  else{
    if (region_articulated_jacobian(0) != 0.0){
      jacs[7] = -((region_articulated_jacobian(0)) / (100 * std::abs(region_articulated_jacobian(0))));
      //jacs[7] += (articulated_point_registration_weight * -(point_articulated_jacobian(0)) / (100 * (std::abs(point_articulated_jacobian(0)) + EPS)));
      //jacs[7] = -(point_articulated_jacobian(0)) / (100 * (std::abs(point_articulated_jacobian(0)) + EPS));
    }
    if (region_articulated_jacobian(1) != 0.0){
      jacs[8] = -0 * ((region_articulated_jacobian(1)) / (100 * std::abs(region_articulated_jacobian(1))));// +(articulated_point_registration_weight * -(point_articulated_jacobian(1)) / (100 * (std::abs(point_articulated_jacobian(1)) + EPS)));
      //jacs[8] = -(point_articulated_jacobian(1)) / (100 * (std::abs(point_articulated_jacobian(1)) + EPS));
    }
    if (region_articulated_jacobian(2) != 0.0f){
      jacs[9] = -((region_articulated_jacobian(2)) / (100 * std::abs(region_articulated_jacobian(2))));// +(articulated_point_registration_weight * -(point_articulated_jacobian(2)) / (100 * (std::abs(point_articulated_jacobian(2)) + EPS)));
      //jacs[9] = -(point_articulated_jacobian(2)) / (100 * (std::abs(point_articulated_jacobian(2)) + EPS));
    }
    if (region_articulated_jacobian(3) != 0.0f){
      jacs[10] = -((region_articulated_jacobian(3)) / (100 * std::abs(region_articulated_jacobian(3))));// +(articulated_point_registration_weight * -(point_articulated_jacobian(3)) / (100 * (std::abs(point_articulated_jacobian(3)) + EPS)));
      //jacs[10] = -(point_articulated_jacobian(3)) / (100 * (std::abs(point_articulated_jacobian(3)) + EPS));
    }

    if (current_model->clasper_1_dislodged){
      ci::app::console() << "Resetting clasper 1!" << std::endl;
      jacs[9] = 0;
    }
    else if (current_model->clasper_2_dislodged){
      ci::app::console() << "Resetting clasper 2!" << std::endl;
      jacs[10] = 0;
    }
  }

  //if (curr_step == NUM_STEPS - 1){
  //auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
  //cv::Vec2f SAMPLED_JACS = ComputeFDJacobianForClasper(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye());
  //jacs[9] = SAMPLED_JACS[0];
  //jacs[10] = SAMPLED_JACS[1];
  //}

  if (head_at_limit){
    if (std::abs(backup_pose[7] + jacs[7]) > max_head_angle){
      ci::app::console() << "Head at limit!!!" << std::endl;
      jacs[7] = 0.0f;
    }
  }


  if (jacs[9] < 0 && jacs[10] < 0){

    ci::app::console() << "Closing claspers" << std::endl;
    //closing
    jacs[8] = 0.0;
    if (joint_closed){
      jacs[9] = 0.0f;
      jacs[10] = 0.0f;
    }


  }
  else if (jacs[9] > 0 && jacs[10] > 0){

    ci::app::console() << "Opening claspers" << std::endl;
    //opening
    jacs[8] = 0.0;
    //jacs[9] = 0.0;
    //jacs[10] = 0.0;


  }
  else if (jacs[9] > 0 && jacs[10] < 0){

    ci::app::console() << "Repointing clasper 1" << std::endl;
    jacs[8] = -0.01;
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;



  }
  else if (jacs[9] < 0 && jacs[10] > 0){

    ci::app::console() << "Repointing clasper 2" << std::endl;

    jacs[8] = 0.01;
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;


  }
  else if (jacs[9] == 0 && jacs[10] == 0){


    ci::app::console() << "Stay still!" << std::endl;

    jacs[8] = 0.00f;
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;
  }
  else if (jacs[9] == 0 && jacs[10] == 0){
    ci::app::console() << "Stay still!" << std::endl;

    jacs[8] = 0.00f;
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;
  }
  else if (jacs[9] > 0 && jacs[10] == 0){
    jacs[8] = -0.01;
    jacs[9] = 0.01;
    jacs[10] = 0.01;
  }

  else if (jacs[9] < 0 && jacs[10] == 0){
    jacs[8] = -0.01;
    jacs[9] = -0.01;
    jacs[10] = -0.01;
  }
  else if (jacs[10] > 0 && jacs[9] == 0){
    jacs[8] = 0.01;
    jacs[9] = 0.01;
    jacs[10] = 0.01;
  }

  else if (jacs[10] < 0 && jacs[9] == 0){
    jacs[8] = 0.01;
    jacs[9] = -0.01;
    jacs[10] = -0.01;

  }
  else{
    ci::app::console() << "THIS SHOULDN'T HAPPEN!!!!" << std::endl;
    ci::app::console() << "jacs[8] = " << jacs[8] << std::endl;
    ci::app::console() << "jacs[9] = " << jacs[9] << std::endl;
    ci::app::console() << "jacs[10] = " << jacs[10] << std::endl;
  }

  if (pointer_at_limit1 && jacs[8] > 0){
    jacs[8] = 0.0f;
  }


  if (pointer_at_limit2 && jacs[8] < 0){
    jacs[8] = 0.0;
  }

  if ((jacs[9] < 0 && jacs[10] < 0) && joint_closed){
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;
  }

  if ((jacs[9] > 0 && jacs[10] > 0) && joint_opened){
    jacs[9] = 0.0f;
    jacs[10] = 0.0f;
  }
}

inline ci::Matrix44f MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation, const std::string &order) {

  float cosx = ci::math<float>::cos(xRotation);
  float cosy = ci::math<float>::cos(yRotation);
  float cosz = ci::math<float>::cos(zRotation);
  float sinx = ci::math<float>::sin(xRotation);
  float siny = ci::math<float>::sin(yRotation);
  float sinz = ci::math<float>::sin(zRotation);

  ci::Matrix33f xRotationMatrix; xRotationMatrix.setToIdentity();
  ci::Matrix33f yRotationMatrix; yRotationMatrix.setToIdentity();
  ci::Matrix33f zRotationMatrix; zRotationMatrix.setToIdentity();

  xRotationMatrix.at(1, 1) = xRotationMatrix.at(2, 2) = cosx;
  xRotationMatrix.at(1, 2) = -sinx;
  xRotationMatrix.at(2, 1) = sinx;

  yRotationMatrix.at(0, 0) = yRotationMatrix.at(2, 2) = cosy;
  yRotationMatrix.at(0, 2) = siny;
  yRotationMatrix.at(2, 0) = -siny;

  zRotationMatrix.at(0, 0) = zRotationMatrix.at(1, 1) = cosz;
  zRotationMatrix.at(0, 1) = -sinz;
  zRotationMatrix.at(1, 0) = sinz;

  ci::Matrix33f r;
  //xyz
  //ci::Matrix33f r = zRotationMatrix * yRotationMatrix * xRotationMatrix;

  //zyx
  if (order == "zyx")
    r = xRotationMatrix * yRotationMatrix * zRotationMatrix;
  else if (order == "xyz")
    r = zRotationMatrix * yRotationMatrix * xRotationMatrix;
  else if (order == "xzy")
    r = yRotationMatrix * zRotationMatrix * xRotationMatrix;
  else
    throw std::runtime_error("");

  ci::Matrix44f rr = r;
  rr.at(3, 3) = 1.0f;
  return rr;

}

float ArticulatedComponentLevelSet::DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points){

  /////////// TO REMOVE ////////////////
  cv::Mat index_image;
  ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
  ////////////////

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> region_rigid_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 4, 1> region_articulated_jacobian = cv::Matx<float, 4, 1>::zeros();

  cv::Matx<float, 7, 1> point_rigid_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 4, 1> point_articulated_jacobian = cv::Matx<float, 4, 1>::zeros();

  ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), region_rigid_jacobian, region_articulated_jacobian, error);
  ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), region_rigid_jacobian, region_articulated_jacobian, error);

  ci::app::console() << "Level set rigid jacs = " << region_rigid_jacobian.t() << std::endl;

  if (track_points)
    ComputeArticulatedPointRegistrationJacobian(current_model, point_rigid_jacobian, point_articulated_jacobian, index_image);

  bool small_steps = false;
  if (curr_step > (0.5 * NUM_STEPS))
    small_steps = true;

  std::vector<float> region_rigid_jacs = ScaleRigidJacobian(region_rigid_jacobian, small_steps);
  std::vector<float> point_rigid_jacs = ScaleRigidJacobian(point_rigid_jacobian, small_steps);
  std::vector<float> jacs;

  for (size_t i = 0; i < number_of_articulated_components_; ++i){
    jacs.push_back(0.0);
  }

  for (size_t v = 0; v < 7; ++v){
    jacs[v] = region_rigid_jacs[v] + point_registration_weight * point_rigid_jacs[v];
  }

  if (optimization_type_ == SAMPLING){
    region_articulated_jacobian = ComputeFDJacobianForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye());
  }

  ApplyClasperLimitStateToJacobian(current_model, jacs, region_articulated_jacobian, point_articulated_jacobian, optimization_type_ == SAMPLING);

  ci::app::console() << "Full JACS = [";
  for (auto &i = jacs.begin(); i != jacs.end(); ++i){
    ci::app::console() << *i << ", ";
  }
  ci::app::console() << "]" << std::endl;

  current_model->UpdatePose(jacs);

  Pose tmp_pose = current_model->GetBasePose();

  if (track_points && (use_global_roll_search_first_ || use_global_roll_search_last_)){
    //if (track_points && curr_step == 0){

    if ((use_global_roll_search_first_ && curr_step == 0) || (use_global_roll_search_last_ && curr_step == NUM_STEPS - 1)){

      Pose mod_pose = current_model->GetBasePose();

      float error_no_change = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      const float error_no_change1 = error_no_change;


      auto offset_01 = MatrixFromIntrinsicEulers(0, 0, 0.03, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_01;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_01 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      auto offset_02 = MatrixFromIntrinsicEulers(0, 0, 0.06, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_02;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_02 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      auto offset_03 = MatrixFromIntrinsicEulers(0, 0, 0.09, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_03;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_03 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      auto offset_neg01 = MatrixFromIntrinsicEulers(0, 0, -0.03, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_neg01;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_neg01 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      auto offset_neg02 = MatrixFromIntrinsicEulers(0, 0, -0.06, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_neg02;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_neg02 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      auto offset_neg03 = MatrixFromIntrinsicEulers(0, 0, -0.09, "zyx");
      mod_pose = (ci::Matrix44f)tmp_pose * offset_neg03;
      current_model->SetBasePose(mod_pose);
      ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), cv::Mat(), cv::Mat(), cv::Mat(), index_image);
      float error_offset_neg03 = point_registration_->GetPointProjectionError(current_model, stereo_camera_->left_eye(), index_image);
      current_model->SetBasePose(tmp_pose);

      ci::app::console() << "Starting from error: " << error_no_change << std::endl;

      if (error_offset_01 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_01);
        error_no_change = error_offset_01;
        ci::app::console() << "Choosing offset +01 with error " << error_offset_01 << " compared with " << error_no_change1 << std::endl;
      }
      if (error_offset_02 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_02);
        error_no_change = error_offset_02;
        ci::app::console() << "Choosing offset +02 with error " << error_offset_02 << " compared with " << error_no_change1 << std::endl;
      }
      if (error_offset_03 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_03);
        error_no_change = error_offset_03;
        ci::app::console() << "Choosing offset +03 with error " << error_offset_03 << " compared with " << error_no_change1 << std::endl;
      }

      if (error_offset_neg01 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_neg01);
        error_no_change = error_offset_neg01;
        ci::app::console() << "Choosing offset -01 with error " << error_offset_neg01 << " compared with " << error_no_change1 << std::endl;
      }
      if (error_offset_neg02 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_neg02);
        error_no_change = error_offset_neg02;
        ci::app::console() << "Choosing offset -02 with error " << error_offset_neg02 << " compared with " << error_no_change1 << std::endl;
      }
      if (error_offset_neg03 < error_no_change){
        current_model->SetBasePose((ci::Matrix44f)tmp_pose * offset_neg03);
        error_no_change = error_offset_neg03;
        ci::app::console() << "Choosing offset -03 with error " << error_offset_neg03 << " compared with " << error_no_change1 << std::endl;
      }

    }

  }

  if (track_points)
    point_registration_->UpdatePointsOnArticulatedModelAfterDerivatives(current_model, index_image);

  return error;

}

void ArticulatedComponentLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 4, 1> &articulated_jacobian, float &error){

  cv::Mat composite_sdf_image, front_intersection_image, back_intersection_image, index_image;

  //Use the articulated components to compute the index_image which is an image where each pixel indexes the articulated component used in the optimization
  ProcessArticulatedSDFAndIntersectionImage(current_model, camera, composite_sdf_image, front_intersection_image, back_intersection_image, index_image);

  cv::Mat sdf_image_1 = components_[0].sdf_image;
  cv::Mat sdf_image_2 = components_[1].sdf_image;
  cv::Mat sdf_image_3 = components_[2].sdf_image;

  current_model->clasper_1_dislodged = false;
  current_model->clasper_2_dislodged = false;

  size_t number_pixels_inner_border_clasper_1 = 0;
  size_t number_pixels_inner_border_clasper_2 = 0;

  float score_pixels_inner_border_clasper_1 = 0;
  float score_pixels_inner_border_clasper_2 = 0;

  //iterate over the multiply component level set (plastic and metal)
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
            ci::app::console() << "NEAREST DIFFERENT NEIGHBOUR FOUND AS BACKGROUND!" << std::endl;
            continue;
          }

          

          error += GetErrorValue(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label, index_image.at<unsigned char>(r, c), 1, 1);

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          float region_agreement = 0;

          if (target_label == 0 || nearest_different_neighbour_label == 0){
            region_agreement = GetBinaryRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);
          }
          else{
            region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label);
          }

          if (std::abs(region_agreement) > 1.0){
            int xxxx = 0;
          }

          region_agreement_im.at<float>(r, c) = region_agreement;

          if (sdf_im_data[i] > 0.0f){

            if (index_image.at<unsigned char>(r, c) == 4){

              number_pixels_inner_border_clasper_1++;
              cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);
              score_pixels_inner_border_clasper_1 += re[1] + re[2] + re[3] + re[4];

            }
            else if (index_image.at<unsigned char>(r, c) == 5){
            
              number_pixels_inner_border_clasper_2++;
              cv::Vec<float, 5> re = classification_image.at < cv::Vec<float, 5> >(r, c);
              score_pixels_inner_border_clasper_2 += re[1] + re[2] + re[3] + re[4];

            }


          }

          int shifted_i = i;

          int closest_r = 0, closest_c = 0;
          //find the closest point on the contour if this point is outside the contour
          if (sdf_im_data[i] < 0.0) {
            if (nearest_different_neighbour_label == 0){
              bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
              if (!found) continue; //should this be allowed to happen?
              shifted_i = closest_r * sdf_image.cols + closest_c;
            }
          }


          std::vector<float> jacobians;
          for (size_t jk = 0; jk < number_of_articulated_components_; ++jk) jacobians.push_back(0.0f);

          const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
          const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

          const cv::Vec3f &front_intersection_point = front_intersection_image.at<cv::Vec3f>(shifted_i);
          const cv::Vec3f &back_intersection_point = back_intersection_image.at<cv::Vec3f>(shifted_i);

          if (front_intersection_point[0] == GL_FAR || back_intersection_point[0] == GL_FAR){
            ci::app::console() << "ERROR-----------------------------------------------" << std::endl;
            ci::app::console() << "Bad intersection point! - " << front_intersection_point << " , " << back_intersection_point << " - " << std::endl;
            ci::app::console() << "At pixel (" << r << ", " << c << ")" << std::endl;
            ci::app::console() << "SDF Value " << sdf_im_data[i] << std::endl;
            ci::app::console() << "Closest pixel (" << closest_r << ", " << closest_c << ")" << std::endl;
            continue;
          }

          //update the jacobian - for component LS sdf determines the component!
          if (camera == stereo_camera_->left_eye())
            UpdateArticulatedJacobian(region_agreement, index_image.at<unsigned char>(shifted_i), sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_point, back_intersection_point, current_model, jacobians);
          else if (camera == stereo_camera_->right_eye())
            UpdateArticulatedJacobianRightEye(region_agreement, index_image.at<unsigned char>(shifted_i), sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacobians);
          else{
            ci::app::console() << "Error, this is an invalid camera!!!" << std::endl;
            throw std::runtime_error("");
          }

          cv::Matx<float, 1, 7> rigid_jacs;
          cv::Matx<float, 1, 4> articulated_jacs;

          //copy the rigid
          for (int j = 0; j < 7; ++j){
            rigid_jacs(j) = jacobians[j];
          }

          for (int j = 0; j < 4; ++j){
            articulated_jacs(j) = jacobians[7 + j];
          }

          float weight = 1.0f;

          if (target_label != 0 && nearest_different_neighbour_label != 0) {
            weight = 0.3;
          }

          rigid_jacobian += (weight * rigid_jacs.t());
          articulated_jacobian += (weight * articulated_jacs.t());

        }
      }
    }
  }

  ci::app::console() << "Score pixels inner border clasper 1 = " << score_pixels_inner_border_clasper_1 << std::endl;
  ci::app::console() << "Number of pixels inner border claser 1  = " << number_pixels_inner_border_clasper_1 << std::endl;
  ci::app::console() << "Score pixels inner border clasper 2 = " << score_pixels_inner_border_clasper_2 << std::endl;
  ci::app::console() << "Number of pixels inner border claser 2  = " << number_pixels_inner_border_clasper_2 << std::endl;

  if (number_pixels_inner_border_clasper_1 > 10 && number_pixels_inner_border_clasper_2 > 10){
    float clasper_1_score = (float)score_pixels_inner_border_clasper_1 / (number_pixels_inner_border_clasper_1);
    float clasper_2_score = (float)score_pixels_inner_border_clasper_2 / (number_pixels_inner_border_clasper_2);

    ci::app::console() << "Clasper 1 score = " << clasper_1_score << std::endl;
    ci::app::console() << "Clasper 2 score = " << clasper_2_score << std::endl;

    if (clasper_1_score > 0.4 && clasper_2_score < 0.05){
      current_model->clasper_2_dislodged = true;
    }
    else if (clasper_2_score > 0.4 && clasper_1_score < 0.05){
      current_model->clasper_1_dislodged = true;
    }
  }

}

void ArticulatedComponentLevelSet::UpdateArticulatedJacobianRightEye(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian){

  if (std::abs(front_intersection_point[2]) < 1.0 || std::abs(back_intersection_point[2]) < 1.0){
    return;
  }

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  ////compute the derivatives w.r.t. left camera pose (this is because we move model w.r.t. left eye)
  ci::Vec3f front_intersection_left_eye = stereo_camera_->TransformPointFromRightToLeft(front_intersection_point);
  ci::Vec3f back_intersection_left_eye = stereo_camera_->TransformPointFromRightToLeft(back_intersection_point);
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_left_eye, frame_idx);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_left_eye, frame_idx);

  //use the 'inverse' of the point transform. as we store the relative orientation (which is already the inverse of the point transform) just use that here.
  const ci::Matrix33f inverse_rotation = stereo_camera_->ciExtrinsicRotation();

  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < jacobian.size(); ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];

    if (sdf == 0.0f){

      const float deriv_x_front =
        front_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_x = dsdf_dx * (fx * z_inv_sq_front * deriv_x_front);

      const float deriv_y_front =
        front_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_y = dsdf_dy * (fy * z_inv_sq_front * deriv_y_front);

      //pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      const float pval = DeltaFunction(sdf) * (deriv_x + deriv_y);

      jacobian[dof] = region_agreement * pval;

    }

    else{

      const float deriv_x_front =
        front_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_x_back =
        back_intersection_point[2] * ((inverse_rotation.at(0, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(0, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(0, 2) * dof_derivatives_back[2]))
        -
        back_intersection_point[0] * ((inverse_rotation.at(2, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_back[2]));

      const float deriv_x = dsdf_dx * ((fx * z_inv_sq_front * deriv_x_front) + (fx * z_inv_sq_back * deriv_x_back));

      const float deriv_y_front =
        front_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_front[2]))
        -
        front_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_front[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_front[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_front[2]));

      const float deriv_y_back =
        back_intersection_point[2] * ((inverse_rotation.at(1, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(1, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(1, 2) * dof_derivatives_back[2]))
        -
        back_intersection_point[1] * ((inverse_rotation.at(2, 0) * dof_derivatives_back[0]) + (inverse_rotation.at(2, 1) * dof_derivatives_back[1]) + (inverse_rotation.at(2, 2) * dof_derivatives_back[2]));
      const float deriv_y = dsdf_dy * ((fy * z_inv_sq_front * deriv_y_front) + (fy * z_inv_sq_back * deriv_y_back));

      //pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      const float pval = DeltaFunction(sdf) * (deriv_x + deriv_y);

      jacobian[dof] = region_agreement * pval;

    }
  }
}

float ArticulatedComponentLevelSet::GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const size_t target_probability, const size_t neighbour_probability, const size_t articulated_component_index, const size_t foreground_size, const size_t background_size) const {

  float v = ComponentLevelSet::GetErrorValue(classification_image, row_idx, col_idx, sdf_value, target_probability, neighbour_probability, foreground_size, background_size);
  return v;

}

void ArticulatedComponentLevelSet::UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian){

  if (std::abs(front_intersection_point[2]) < 1.0 || std::abs(back_intersection_point[2]) < 1.0){
    return;
  }

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  if (front_intersection_point[2] == GL_FAR || back_intersection_point[2] == GL_FAR) {
    ci::app::console() << "Articulated Jacobian intersection point is bad" << front_intersection_point << std::endl;
    throw std::runtime_error("HERE");
  }

  //get the frame index for the composite sdf map
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_point, frame_idx);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_point, frame_idx);


  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < jacobian.size(); ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];



    if (sdf == 0.0f){
      float pval = dsdf_dx * (fx * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[0]) - (front_intersection_point[0] * dof_derivatives_front[2]))));
      pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))));
      pval *= DeltaFunction(sdf);

      jacobian[dof] = region_agreement * pval;

    }
    else{
      //actually compute the cost function equation for the degree of freedom in question
      float pval = dsdf_dx * (fx * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[0]) - (front_intersection_point[0] * dof_derivatives_front[2]))) + fx * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[0]) - (back_intersection_point[0] * dof_derivatives_back[2]))));
      pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      pval *= DeltaFunction(sdf);

      jacobian[dof] = region_agreement * pval;

    }
  }

}

void ArticulatedComponentLevelSet::ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &composite_sdf_image, cv::Mat &composite_front_intersection_image, cv::Mat &composite_back_intersection_image, cv::Mat &frame_idx_image){

  //have sdfs for each component (one for black shaft and one for tip)
  //have sdfs for each articulated part

  //main optimization is done over the components so we iterate first over the shaft component part
  //this will use the index image to index the joint component needed for optimization
  //shouldn

  StereoPWP3D::ProcessSDFAndIntersectionImage(mesh, camera, composite_sdf_image, composite_front_intersection_image, composite_back_intersection_image);
  frame_idx_image = cv::Mat(composite_sdf_image.size(), CV_8UC1);

  std::vector<Node *> nodes;

  size_t idx = 1;
  Node *p = mesh->GetModel().get();

  nodes.push_back(p);

  while (1){

    Node *p1 = p->GetChildByIdx(idx);
    if (p1 == nullptr) break;
    idx += 1;

    nodes.push_back(p1);

  }


  std::vector<ComponentData> model_parts;

  for (auto node : nodes){

    for (auto nodeA : nodes){
      nodeA->SetDraw(false);
    }

    node->SetDraw(true);

    model_parts.push_back(ComponentData());

    model_parts.back().model_ = node;

    if (!node->HasMesh()) continue;

    cv::Mat sdf_image, front_intersection_image, back_intersection_image;
    StereoPWP3D::ProcessSDFAndIntersectionImage(mesh, camera, sdf_image, front_intersection_image, back_intersection_image);
    //ComponentLevelSet::ProcessSDFAndIntersectionImage(mesh, camera, front_intersection_image, back_intersection_image);



    model_parts.back().sdf_image_ = sdf_image;



    model_parts.back().front_intersection_image_ = front_intersection_image;
    model_parts.back().back_intersection_image_ = back_intersection_image;

  }

  cv::Mat &shaft_image = model_parts[0].front_intersection_image_;
  cv::Mat &head_image = model_parts[1].front_intersection_image_;
  cv::Mat &clasper1_image = model_parts[4].front_intersection_image_;
  cv::Mat &clasper2_image = model_parts[5].front_intersection_image_;


  for (auto nodeA : nodes){
    nodeA->SetDraw(true);
  }

  cv::Mat save_image = cv::Mat::zeros(composite_sdf_image.size(), CV_8UC1);

  for (int r = 0; r < composite_sdf_image.rows; ++r){

    for (int c = 0; c < composite_sdf_image.cols; ++c){

      float min_dist = GL_FAR;

      for (size_t t = 0; t < model_parts.size(); ++t){

        if (model_parts[t].model_->HasMesh() == 0) continue;

        if (model_parts[t].front_intersection_image_.at<cv::Vec3f>(r, c) != cv::Vec3f(GL_FAR, GL_FAR, GL_FAR)){

          float dist = std::abs(model_parts[t].front_intersection_image_.at<cv::Vec3f>(r, c)[2]);
          if (dist < min_dist){
            min_dist = dist;
            save_image.at<unsigned char>(r, c) = model_parts[t].model_->GetIdx();
          }

        }
        else if (composite_sdf_image.at<float>(r, c) < 0){
          if (composite_sdf_image.at<float>(r, c) == model_parts[t].sdf_image_.at<float>(r, c)){
            save_image.at<unsigned char>(r, c) = t;
          }
        }
      }


      frame_idx_image.at<unsigned char>(r, c) = 255;

      if (composite_sdf_image.at<float>(r, c) >= 0.0){
        //for points inside

        float min_dist = GL_FAR;

        for (size_t t = 0; t < model_parts.size(); ++t){

          if (model_parts[t].model_->HasMesh() == 0) continue;

          if (model_parts[t].front_intersection_image_.at<cv::Vec3f>(r, c) != cv::Vec3f(GL_FAR, GL_FAR, GL_FAR)){

            float dist = std::abs(model_parts[t].front_intersection_image_.at<cv::Vec3f>(r, c)[2]);
            if (dist < min_dist){
              min_dist = dist;
              frame_idx_image.at<unsigned char>(r, c) = model_parts[t].model_->GetIdx();
            }

          }
        }

      }
      else{

        //float min_dist = std::numeric_limits<float>::max();

        //for (size_t t = 0; t < model_parts.size(); ++t){
        //
        //  if (model_parts[t].model_->HasMesh() == 0) continue;

        //  float dist = std::abs(model_parts[t].sdf_image_.at<float>(r, c));
        //  if (dist < min_dist){
        //    min_dist = dist;
        //    frame_idx_image.at<unsigned char>(r, c) = model_parts[t].model_->GetIdx();
        //  }

        //}

        frame_idx_image.at<unsigned char>(r, c) = 255;

      }

    }

  }

  double min, max;
  cv::Mat sdf_scaled(composite_sdf_image.size(), CV_8UC1);
  cv::minMaxLoc(composite_sdf_image, &min, &max);

  for (int r = 0; r < composite_sdf_image.rows; ++r){
    for (int c = 0; c < composite_sdf_image.cols; ++c){
      sdf_scaled.at<unsigned char>(r, c) = 255 * ((composite_sdf_image.at<float>(r, c) - min) / (max - min));
    }
  }

  cv::imwrite("c:/tmp/composite_sdf_image.png", sdf_scaled);

  cv::minMaxLoc(save_image, &min, &max);

  for (int r = 0; r < composite_sdf_image.rows; ++r){
    for (int c = 0; c < composite_sdf_image.cols; ++c){
      sdf_scaled.at<unsigned char>(r, c) = 255 * (((int)save_image.at<unsigned char>(r, c) - min) / (max - min));
    }
  }

  cv::imwrite("c:/tmp/frame_index_image.png", sdf_scaled);


  //need to reset the components... the previous articulated rendering messes them up...
  cv::Mat front_intersection_image_, back_intersection_image_;
  ComponentLevelSet::ProcessSDFAndIntersectionImage(mesh, camera, front_intersection_image_, back_intersection_image_);


}
