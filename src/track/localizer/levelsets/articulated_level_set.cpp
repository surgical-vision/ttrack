#include <cinder/app/App.h>
#include <numeric>

#include "../../../include/ttrack/track/localizer/levelsets/articulated_level_set.hpp"
#include "../../../include/ttrack/constants.hpp"

using namespace ttrk;

void ArticulatedComponentLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;

  if (curr_step == NUM_STEPS || first_run_) {

    cv::Mat front_intersection_image, back_intersection_image, front_normal_image, frame_idx_image;
    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), front_intersection_image, back_intersection_image, front_normal_image, frame_idx_image);

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

  float error = DoAlignmentStep(current_model, true);
  UpdateWithErrorValue(error);
  errors_.push_back(error);


 



}


float ArticulatedComponentLevelSet::DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points){

  float error = 0.0f;
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> rigid_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 4, 1> articulated_jacobian = cv::Matx<float, 4, 1>::zeros();

  const int max_pose_param = 4; 

  if (current_pose_param_ >= max_pose_param) current_pose_param_ = 0;

  if (current_rigid_step_ >= num_rigid_steps_) {
    current_rigid_step_ = 0;
    current_pose_param_++;
  }

  ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), rigid_jacobian, articulated_jacobian, error);
  ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), rigid_jacobian, articulated_jacobian, error);

  cv::Matx<float, 7, 7> dmp; //not used.
  if (track_points)
    ComputeLKJacobian(current_model, rigid_jacobian, dmp);
  
  std::vector<float> rigid_jacs = ScaleRigidJacobian(rigid_jacobian);
  std::vector<float> jacs;

  for (size_t i = 0; i < number_of_articulated_components_; ++i){
    jacs.push_back(0.0);
  }

  if (current_pose_param_ == 0){
    for (size_t v = 0; v < 7; ++v){
      jacs[v] = rigid_jacs[v];
    }
  }


  if (current_pose_param_ == 1){
    jacs[7] = -(articulated_jacobian(0) / (100 * std::abs(articulated_jacobian(0))));
  }
  else if (current_pose_param_ == 2){
    jacs[8] = -(articulated_jacobian(1) / (100 * std::abs(articulated_jacobian(1))));
  }
  else if (current_pose_param_ == 3){

    jacs[9] = -(articulated_jacobian(2) / (100 * std::abs(articulated_jacobian(2))));

    jacs[10] = -(articulated_jacobian(3) / (100 * std::abs(articulated_jacobian(3))));

    auto j_sum = jacs[9] + jacs[10];

    jacs[9] = j_sum;
    jacs[10] = j_sum;
  }

  current_model->UpdatePose(jacs);

  if (track_points)
    lk_tracker_->UpdatePointsOnModelAfterDerivatives(current_model->GetBasePose());

  return error;

}

void ArticulatedComponentLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 4, 1> &articulated_jacobian, float &error){

  cv::Mat composite_sdf_image, front_intersection_image, back_intersection_image, index_image;
  
  ProcessArticulatedSDFAndIntersectionImage(current_model, camera, composite_sdf_image, front_intersection_image, back_intersection_image, index_image);

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
            continue;
          }

          error += GetErrorValue(classification_image, r, c, sdf_im_data[i], target_label, nearest_different_neighbour_label, index_image.at<int>(r, c));

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


          std::vector<float> jacobians;
          for (size_t i = 0; i < number_of_articulated_components_; ++i) jacobians.push_back(0.0f);

          const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
          const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

          //update the jacobian - for component LS sdf determines the component!
          if (camera == stereo_camera_->left_eye())
            UpdateArticulatedJacobian(region_agreement, index_image.at<int>(shifted_i), sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacobians);
          else if (camera == stereo_camera_->right_eye())
            UpdateArticulatedJacobianRightEye(region_agreement, index_image.at<int>(shifted_i), sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacobians);
          else
            throw std::runtime_error("");

          cv::Matx<float, 1, 7> rigid_jacs;
          cv::Matx<float, 1, 4> articulated_jacs;

          //copy the rigid
          for (int j = 0; j < 7; ++j){
            rigid_jacs(j) = jacobians[j];
          }

          for (int j = 0; j < 4; ++j){
            articulated_jacs(j) = jacobians[7 + j];
          }

          rigid_jacobian += rigid_jacs.t();
          articulated_jacobian += articulated_jacs.t();

        }
      }
    }
  }
}

void ArticulatedComponentLevelSet::UpdateArticulatedJacobianRightEye(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian){


}

float ArticulatedComponentLevelSet::GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const size_t target_probability, const size_t neighbour_probability, const size_t articulated_component_index) const {

  //
  return ComponentLevelSet::GetErrorValue(classification_image, row_idx, col_idx, sdf_value, target_probability, neighbour_probability);


}



void ArticulatedComponentLevelSet::UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian){

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  if (front_intersection_point[2] == GL_FAR || back_intersection_point[2] == GL_FAR) throw std::runtime_error("HERE");

  //get the frame index for the composite sdf map
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_point, frame_idx);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_point, frame_idx);


  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < jacobian.size() ; ++dof){

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
  
  for (int r = 0; r < composite_sdf_image.rows; ++r){

    for (int c = 0; c < composite_sdf_image.cols; ++c){

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

}
