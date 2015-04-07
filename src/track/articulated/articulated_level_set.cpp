#include <cinder/app/App.h>
#include <numeric>

#include "../../../include/ttrack/track/articulated/articulated_level_set.hpp"
#include "../../../include/ttrack/constants.hpp"

using namespace ttrk;

void ArticulatedLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  if (curr_step == NUM_STEPS) curr_step = 0;

  ++curr_step;

  frame_ = frame;

  //for prototyping the articulated jacs, we use a cv::Matx. this will be flattened for faster estimation later
  cv::Matx<float, 7, 1> rigid_jacobian = cv::Matx<float, 7, 1>::zeros();
  cv::Matx<float, 7, 7> rigid_hessian_approx = cv::Matx<float, 7, 7>::zeros();

  cv::Matx<float, 4, 1> articulated_jacobian = cv::Matx<float, 4, 1>::zeros();
  cv::Matx<float, 4, 4> articulated_hessian_approx = cv::Matx<float, 4, 4>::zeros();

  const int max_pose_param = 4; // 0 = do rigid, 1 = wp, 2 = wy, 3 = cr

  if (current_pose_param_ >= max_pose_param) current_pose_param_ = 0;

  if (current_rigid_step_ >= num_rigid_steps_) {
    current_rigid_step_ = 0;
    current_pose_param_++;
  }

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  float left_error = 0.0f, right_error = 0.0f;
  ComputeJacobiansForEye(stereo_frame->GetLeftClassificationMap(), current_model, stereo_camera_->left_eye(), rigid_jacobian, rigid_hessian_approx, articulated_jacobian, articulated_hessian_approx, left_error);
  //ComputeJacobiansForEye(stereo_frame->GetRightClassificationMap(), current_model, stereo_camera_->right_eye(), rigid_jacobian, rigid_hessian_approx, articulated_jacobian, articulated_hessian_approx, left_error);

  UpdateWithErrorValue(left_error + right_error);

  if (previous_error_value_ < 0) {
    previous_error_value_ = left_error + right_error;
  }
  else{
    if (left_error + right_error > previous_error_value_ && current_pose_param_ == 0){
      current_pose_param_++;
    }
  }

  rigid_jacobian = rigid_hessian_approx.inv() * rigid_jacobian;

  std::vector<float> jacs(PRECISION, 0);

  if (current_pose_param_ == 0){
    for (size_t v = 0; v < 7; ++v){
      jacs[v] = -rigid_jacobian(v);
    }
  }


  if (current_pose_param_ == 1){
    //jacs[7] = -20*(j_sum_arti(0) / (j_sum_arti(0)*j_sum_arti(0)));
    jacs[7] = -(articulated_jacobian(0) / (100 * std::abs(articulated_jacobian(0))));
  }

  if (current_pose_param_ == 2){
    //jacs[8] = -20*(j_sum_arti(1) / (j_sum_arti(1)*j_sum_arti(1)));
    jacs[8] = -(articulated_jacobian(1) / (100 * std::abs(articulated_jacobian(1))));

  }

  if (current_pose_param_ == 3){

    //jacs[9] = -20 * (j_sum_arti(2) / (j_sum_arti(2)*j_sum_arti(2)));
    jacs[9] = -(articulated_jacobian(2) / (100 * std::abs(articulated_jacobian(2))));

    jacs[10] = -(articulated_jacobian(3) / (100 * std::abs(articulated_jacobian(3))));

    auto j_sum = jacs[9] + jacs[10];

    jacs[9] = j_sum;
    jacs[10] = j_sum;
    // jacs[10] = 20 * (j_sum_arti(2) / (j_sum_arti(2)*j_sum_arti(2)));//should be 'negative'
  }


  ci::app::console() << "Jacobian = [";
  for (size_t v = 0; v < jacs.size(); ++v){
    ci::app::console() << jacs[v] << ",";
  }
  ci::app::console() << "]" << std::endl;

  if (current_pose_param_ == 0){
    current_rigid_step_++;
  }
  else{
    current_pose_param_++;
  }

  current_model->UpdatePose(jacs);

}

void ArticulatedLevelSet::ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &rigid_jacobian, cv::Matx<float, 7, 7> &rigid_hessian_approx, cv::Matx<float, 4, 1> &articulated_jacobian, cv::Matx<float, 4, 4> &articulated_hessian_approx, float &error){

  cv::Mat sdf_image, front_intersection_image, back_intersection_image, index_image;
  
  ProcessArticulatedSDFAndIntersectionImage(current_model, camera, sdf_image, front_intersection_image, back_intersection_image, index_image);

  float fg_area = 0, bg_area = 0;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;
  unsigned char *index_image_data = (unsigned char *)index_image.data;

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //-log(H * P_f + (1-H) * P_b)
        error += GetErrorValue(classification_image, r, c, sdf_im_data[i], index_image_data[i], fg_area, bg_area);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], index_image_data[i]);

        int shifted_i = i;

        //find the closest point on the contour if this point is outside the contour
        if (sdf_im_data[i] < 0.0) {
          int closest_r, closest_c;
          bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
          if (!found) continue; //should this be allowed to happen?
          shifted_i = closest_r * sdf_image.cols + closest_c;
        }

        cv::Matx<float, 1, PRECISION> jacs;
        for (int j = 0; j < PRECISION; ++j){
          jacs(j) = 0.0f;
        }

        const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
        const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

        //update the jacobian
        UpdateArticulatedJacobian(region_agreement, index_image_data[shifted_i], sdf_im_data[i], dsdf_dx, dsdf_dy, camera->Fx(), camera->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);

        cv::Matx<float, 1, 7> rigid_jacs;
        cv::Matx<float, 1, 4> articulated_jacs;

        //copy the rigid
        for (int j = 0; j < 7; ++j){
          rigid_jacs(j) = jacs(j);
        }

        for (int j = 0; j < 4; ++j){
          articulated_jacs(j) = jacs(7+j);
        }

        rigid_jacobian += rigid_jacs.t();
        rigid_hessian_approx += (rigid_jacs.t() * rigid_jacs);

        articulated_jacobian += articulated_jacs.t();
        articulated_hessian_approx += (articulated_jacs.t() * articulated_jacs);

      }
    }
  }

}

float ArticulatedLevelSet::GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf, const int target_label) const {

  const float heaviside_value = HeavisideFunction(sdf);

  const float Pf = classification_image.at<cv::Vec4f>(r, c)[1]; //returns foreground pixel likelihood -foreground class
  const float Pb = classification_image.at<cv::Vec4f>(r, c)[0]; //returns background pixel likelihood

  return (Pf - Pb) / ((heaviside_value*Pf) + ((1 - heaviside_value)*Pb));

}


void ArticulatedLevelSet::UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, cv::Matx<float, 1, PRECISION> &jacobian){

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  if (front_intersection_point[2] == GL_FAR || back_intersection_point[2] == GL_FAR) throw std::runtime_error("HERE");

  //get the frame index for the composite sdf map

  //model->GetModel()->ComputeJacobianForPoint(ci::Vec3f(front_intersection_point[0], front_intersection_point[1], front_intersection_point[2]), frame_idx, front_jacs);
  //model->GetModel()->ComputeJacobianForPoint(ci::Vec3f(back_intersection_point[0], back_intersection_point[1], back_intersection_point[2]), frame_idx, back_jacs);
  std::vector<ci::Vec3f> front_jacs = model->ComputeJacobian(front_intersection_point, frame_idx);
  std::vector<ci::Vec3f> back_jacs = model->ComputeJacobian(back_intersection_point, frame_idx);

  //assert(front_jacs.size() == 12); //for now

  //for each degree of freedom, compute the jacobian update
  for (size_t dof = 0; dof < PRECISION/*front_jacs.size()*/; ++dof){

    const ci::Vec3f &dof_derivatives_front = front_jacs[dof];
    const ci::Vec3f &dof_derivatives_back = back_jacs[dof];

    if (sdf == 0.0f){
      float pval = dsdf_dx * (fx * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[0]) - (front_intersection_point[0] * dof_derivatives_front[2]))));
      pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))));
      pval *= DeltaFunction(sdf);

      jacobian(dof) = region_agreement * pval;

    }
    else{
      //actually compute the cost function equation for the degree of freedom in question
      float pval = dsdf_dx * (fx * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[0]) - (front_intersection_point[0] * dof_derivatives_front[2]))) + fx * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[0]) - (back_intersection_point[0] * dof_derivatives_back[2]))));
      pval += dsdf_dy * (fy * (z_inv_sq_front*((front_intersection_point[2] * dof_derivatives_front[1]) - (front_intersection_point[1] * dof_derivatives_front[2]))) + fy * (z_inv_sq_back*((back_intersection_point[2] * dof_derivatives_back[1]) - (back_intersection_point[1] * dof_derivatives_back[2]))));
      pval *= DeltaFunction(sdf);

      jacobian(dof) = region_agreement * pval;
    }
  }

}

void ArticulatedLevelSet::ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &composite_sdf_image, cv::Mat &composite_front_intersection_image, cv::Mat &composite_back_intersection_image, cv::Mat &frame_idx_image){

  ProcessSDFAndIntersectionImage(mesh, camera, composite_sdf_image, composite_front_intersection_image, composite_back_intersection_image);
  frame_idx_image = cv::Mat(cv::Size(frame_->cols(), frame_->rows()), CV_8UC1);

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
    ProcessSDFAndIntersectionImage(mesh, camera, sdf_image, front_intersection_image, back_intersection_image);

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

void ArticulatedLevelSet::SetParameters(boost::shared_ptr<Model> current_model, double const* const*parameters){

  std::vector<float> f_params;
  //for (int i = 0; i < PRECISION; ++i){

  //  f_params.push_back(parameters[i]);

  //}
  for (int i = 0; i < 3; ++i){
    f_params.push_back((float)parameters[0][i]);
  }

  auto v = current_model->GetBasePose().GetPose();

  for (int i = 0; i < 4; ++i){
    f_params.push_back((float)v[3 + i]);
  }
  for (int i = 0; i < 5; ++i){
    f_params.push_back((float)parameters[2][i]);
  }

  current_model->SetPose(f_params);

}

cv::Matx<float, NUM_RESIDUALS, PRECISION> ArticulatedLevelSet::ComputeJacobians(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image){

  cv::Matx<float, NUM_RESIDUALS, PRECISION > jacobians = cv::Matx<float, NUM_RESIDUALS, PRECISION>::zeros();

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;
  unsigned char *index_image_data = (unsigned char *)index_image.data;

  float fg_area = 0, bg_area = 0;
  size_t contour_area = 0;
  ComputeAreas(sdf_image, fg_area, bg_area, contour_area);

  //float error_value = 0.0f;

  int current_row = 0;

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        const float cost_value = GetErrorValue(classification_image, r, c, sdf_im_data[i], index_image_data[i], fg_area, bg_area);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], index_image_data[i]);

        //find the closest point on the contour if this point is outside the contour
        if (sdf_im_data[i] < 0.0) {
          int closest_r, closest_c;
          bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
          if (!found) continue; //should this be allowed to happen?
          i = closest_r * sdf_image.cols + closest_c;
        }

        cv::Matx<float, 1, PRECISION> jacs;
        for (int j = 0; j < PRECISION; ++j){
          jacs(j) = 0.0f;
        }

        const float dsdf_dx = 0.5f*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
        const float dsdf_dy = 0.5f*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

        //update the jacobian
        UpdateArticulatedJacobian(region_agreement, index_image_data[i], sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(i), back_intersection_image.at<cv::Vec3f>(i), current_model, jacs);

        if (current_row >= NUM_RESIDUALS){

          current_row++;
          continue;

        }

        for (int a = 0; a < PRECISION; ++a){
          jacobians(current_row, a) = cost_value * jacs(a);
        }

        current_row++;

      }
    }
  }

  for (; current_row < NUM_RESIDUALS; ++current_row){
    for (int a = 0; a < PRECISION; ++a){
      jacobians(current_row, a) = jacobians(0, a);
    }
  }

  return jacobians;

}

cv::Matx<float, PRECISION, 1> ArticulatedLevelSet::ComputeJacobiansSummed(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image){

  cv::Matx<float, PRECISION, 1> j_sum = cv::Matx<float, PRECISION, 1>::zeros();

  cv::Mat dsdf_dx, dsdf_dy;
  cv::Scharr(sdf_image, dsdf_dx, CV_32FC1, 1, 0);
  cv::Scharr(sdf_image, dsdf_dy, CV_32FC1, 0, 1);

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;
  float *dsdf_dx_data = (float *)dsdf_dx.data;
  float *dsdf_dy_data = (float *)dsdf_dy.data;
  unsigned char *index_image_data = (unsigned char *)index_image.data;

  //float error_value = 0.0f;

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(classification_image, r, c, sdf_im_data[i], index_image_data[i]);

        //find the closest point on the contour if this point is outside the contour
        if (sdf_im_data[i] < 0.0) {
          int closest_r, closest_c;
          bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
          if (!found) continue; //should this be allowed to happen?
          i = closest_r * sdf_image.cols + closest_c;
        }

        cv::Matx<float, 1, PRECISION> jacs;
        for (int j = 0; j < PRECISION; ++j){
          jacs(j) = 0.0f;
        }

        //update the jacobian
        UpdateArticulatedJacobian(region_agreement, index_image_data[i], sdf_im_data[i], dsdf_dx_data[i], dsdf_dy_data[i], stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(i), back_intersection_image.at<cv::Vec3f>(i), current_model, jacs);
        j_sum += jacs.t();

      }
    }
  }

  return j_sum;

}
