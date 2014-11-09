#include "../../../include/track/articulated/articulated_level_set.hpp"

using namespace ttrk;


void ArticulatedLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;

  //index images give the index from the 
  cv::Mat left_sdf_image, left_front_intersection, left_back_intersection, left_index_image;
  cv::Mat right_sdf_image, right_front_intersection, right_back_intersection, right_index_image;

  //load multiclass detection module where each pixels is labelled (0:background, 1: shaft, 2: head, 3: clasper)

  //models is currently a tree of components

  //create a SDF and intersection for each component

  //possible to include some interior surface information by including the signed distnace function around the edge between the instrument shaft and instrument head

  //do normal pwp3d cost function iterating over the pixels


  //iterate until steps or convergences
  for (size_t step = 0; step < NUM_STEPS; ++step){

    //for prototyping the articulated jacs, we use a vector. this will be flattened for faster estimation later
    std::vector<float> jacs;

    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, left_front_intersection, left_back_intersection,left_index_image);
    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, right_front_intersection, right_back_intersection, right_index_image);

    cv::Mat dsdf_dx, dsdf_dy;
    cv::Scharr(left_sdf_image, dsdf_dx, CV_32FC1, 1, 0);
    cv::Scharr(left_sdf_image, dsdf_dy, CV_32FC1, 0, 1);

    cv::Mat &classification_image = frame_->GetClassificationMapROI();
    cv::Mat &front_intersection_image = left_front_intersection;
    cv::Mat &back_intersection_image = right_front_intersection;
    cv::Mat &sdf_image = left_sdf_image;
    cv::Mat &index_image = left_index_image;

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;
    float *dsdf_dx_data = (float *)dsdf_dx.data;
    float *dsdf_dy_data = (float *)dsdf_dy.data;
    unsigned char *index_image_data = (unsigned char *)index_image.data;

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c){

        int i = r*classification_image.cols + c;

        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(r, c, sdf_im_data[i], index_image_data[i]);

          //find the closest point on the contour if this point is outside the contour
          if (sdf_im_data[i] < 0.0) {
            int closest_r, closest_c;
            bool found = FindClosestIntersection(sdf_im_data, r, c, sdf_image.rows, sdf_image.cols, closest_r, closest_c);
            if (!found) continue; //should this be allowed to happen?
            i = closest_r * sdf_image.cols + closest_c;
          }

          //update the jacobian
          UpdateArticulatedJacobian(region_agreement, index_image_data[i], sdf_im_data[i], dsdf_dx_data[i], dsdf_dy_data[i], stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), cv::Vec3f(&front_intersection_data[i * 3]), cv::Vec3f(&back_intersection_data[i * 3]), current_model, jacs);

        }
      }
    }

    current_model->UpdatePose(jacs);

  }

  //for each pixel find the sdf that it corrsponds to 

  //each component can have a colour model (one against all classification)

  //region agreement is computed as same

  //3d intersection values need to be in camera frame and LOCAL coordinate frame of the geometry

  //update the correct part of the jacobian with the derivative

  //shaft -> rigid se3 
  //head -> wrist pitch
  //jointly solve claspers and wrist yaw

}

float ArticulatedLevelSet::GetRegionAgreement(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const{

  const float pixel_probability = frame_->GetClassificationMapROI().at<float>(row_idx, col_idx);
  const float heaviside_value = HeavisideFunction(sdf_value);

  return (2.0f*pixel_probability - 1.0f) / (heaviside_value*pixel_probability + (1.0f - heaviside_value)*(1.0f - pixel_probability));

}


void ArticulatedLevelSet::UpdateArticulatedJacobian(const float region_agreement, const int frame_idx, const float sdf, const float dsdf_dx, const float dsdf_dy, const float fx, const float fy, const cv::Vec3f &front_intersection_point, const cv::Vec3f &back_intersection_point, const boost::shared_ptr<const Model> model, std::vector<float> &jacobian){

  const float z_inv_sq_front = 1.0f / (front_intersection_point[2] * front_intersection_point[2]);
  const float z_inv_sq_back = 1.0f / (back_intersection_point[2] * back_intersection_point[2]);

  //get the frame index for the composite sdf map
  std::vector<ci::Vec3f> front_jacs, back_jacs;
  model->GetModel()->ComputeJacobianForPoint(ci::Vec3f(front_intersection_point[0], front_intersection_point[1], front_intersection_point[2]), frame_idx, front_jacs);
  model->GetModel()->ComputeJacobianForPoint(ci::Vec3f(back_intersection_point[0], back_intersection_point[1], back_intersection_point[2]), frame_idx, back_jacs);

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


void ArticulatedLevelSet::ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &composite_sdf_image, cv::Mat &composite_front_intersection_image, cv::Mat &composite_back_intersection_image, cv::Mat &frame_idx_image){

  std::vector<Node::Ptr> nodes;

  size_t idx = 1;
  Node::Ptr p = mesh->GetModel();

  nodes.push_back(p);

  while (1){

    Node::Ptr p1 = p->GetChildByIdx(idx);
    if (p1 == nullptr) break;
    idx += 1;

    nodes.push_back(p1);

  }

  std::vector<ComponentData> thangs;

  for (auto node : nodes){

    for (auto nodeA : nodes){
      nodeA->SetDraw(false);
    }

    node->SetDraw(true);

    thangs.push_back(ComponentData());

    thangs.back().model_ = node;

    if (!node->HasMesh()) continue;

    StereoPWP3D tracker(stereo_camera_);
    tracker.SetFrame(frame_);

    cv::Mat sdf_image, front_intersection_image, back_intersection_image;
    tracker.ProcessSDFAndIntersectionImage(mesh, camera, sdf_image, front_intersection_image, back_intersection_image);

    thangs.back().sdf_image_ = sdf_image;
    thangs.back().front_intersection_image_ = front_intersection_image;
    thangs.back().back_intersection_image_ = back_intersection_image;

  }

  composite_sdf_image = cv::Mat(cv::Size(frame_->cols(), frame_->rows()), CV_32FC1);
  frame_idx_image = cv::Mat(cv::Size(frame_->cols(), frame_->rows()), CV_8UC1);

  for (int r = 0; r < composite_sdf_image.rows; ++r){

    for (int c = 0; c < composite_sdf_image.cols; ++c){


      float min_val = std::numeric_limits<float>::max();

      for (size_t t = 0; t < thangs.size(); ++t){

        if (thangs[t].model_->HasMesh() == 0) continue;

        float val = std::abs(thangs[t].sdf_image_.at<float>(r, c));
        if (val <= min_val){
          min_val = std::abs(thangs[t].sdf_image_.at<float>(r, c));
          composite_sdf_image.at<float>(r, c) = thangs[t].sdf_image_.at<float>(r, c);
          frame_idx_image.at<unsigned char>(r, c) = thangs[t].model_->GetIdx();
        }

      }

    }

  }

}

