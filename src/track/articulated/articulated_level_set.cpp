#include <cinder/app/App.h>
#include <numeric>


#include "../../../include/track/articulated/articulated_level_set.hpp"
#include "../../../include/constants.hpp"


using namespace ttrk;



ArticulatedLevelSet *GLOBAL_ALS = 0x0;

//
//void ArticulatedLevelSetSolverSingleResidual::TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame){
//
//  frame_ = frame;
//  current_model_ = model;
//
//  std::vector<float> pose_parameters;
//  model->GetPose(pose_parameters);
//
//  //initalise
//  double **parameters = new double*[3];
//  parameters[0] = new double[3];
//  parameters[1] = new double[4];
//  parameters[2] = new double[5];
//
//  parameters[0][0] = pose_parameters[0];
//  parameters[0][1] = pose_parameters[1];
//  parameters[0][2] = pose_parameters[2];
//
//  parameters[1][0] = pose_parameters[3];
//  parameters[1][1] = pose_parameters[4];
//  parameters[1][2] = pose_parameters[5];
//  parameters[1][3] = pose_parameters[6];
//
//  parameters[2][0] = pose_parameters[7];
//  parameters[2][1] = pose_parameters[8];
//  parameters[2][2] = pose_parameters[9];
//  parameters[2][3] = pose_parameters[10];
//  parameters[2][4] = pose_parameters[11];
//
//  GLOBAL_ALS = new ArticulatedLevelSet(stereo_camera_);
//
//  ceres::Problem problem;
//
//  //option 1 use this version with anyltic derivatives
//  //supply initial problem guess - parameters
//  //problem.AddResidualBlock(this, nullptr, parameters); 
//
//  //option 2 use automatic derivatives
//  
//  //option 3 add each residual as a separate cost function
//  //to do this one need to iterate over sdf image and get indexes of all pixels we want to process
//  //also need to figure out how we can 'refresh' the model at each loop
//  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<LevelSetResidual, 1, 3, 4, 1, 1, 1, 1, 1>(new LevelSetResidual();
//  //  problem.AddResidualBlock(cost_function, nullptr, parameters);
//  //}
//
//
//  ceres::Solver::Options options;
//  options.linear_solver_type = ceres::DENSE_QR;
//  options.minimizer_progress_to_stdout = false;
//  options.update_state_every_iteration = true;
//  //options.check_gradients = true;
//  //options.jacobi_scaling = true;
//  //options.minimizer_type = ceres::TRUST_REGION;
//  options.minimizer_type = ceres::LINE_SEARCH;
//  options.line_search_direction_type = ceres::STEEPEST_DESCENT;
//  ceres::Solver::Summary summary;
//  ceres::Solve(options, &problem, &summary);
//
//  ci::app::console() << summary.FullReport() << std::endl;
//
//
//  delete GLOBAL_ALS;
//  delete[] parameters[0];
//  delete[] parameters[1];
//  delete[] parameters[2];
//  delete[] parameters;
//
//
//}

//compute jacobians by self if using the anyltic derivative solver
bool ArticulatedLevelSetSolver::Evaluate(double const* const* parameters, double *residuals, double **jacobians) const{

  GLOBAL_ALS->SetFrame(frame_);
  cv::Mat left_sdf_image, left_front_intersection, left_back_intersection, left_index_image, left_classification_image;
  left_classification_image = frame_->GetClassificationMapROI();

  ci::app::console() << "Updated parameters = [";
  for (int i = 0; i < parameter_block_sizes().size(); ++i){

    int num_params_in_block = parameter_block_sizes()[i];

    for (int c = 0; c < num_params_in_block; ++c){

      ci::app::console() << parameters[i][c] << ", ";

    }

  }

  ci::app::console() << "]" << std::endl;
  
  GLOBAL_ALS->SetParameters(current_model_,parameters);

  GLOBAL_ALS->ProcessArticulatedSDFAndIntersectionImage(current_model_, stereo_camera_->left_eye(), left_sdf_image, left_front_intersection, left_back_intersection, left_index_image);

  float *sdf_im_data = (float *)left_sdf_image.data;
  unsigned char *index_image_data = (unsigned char *)left_index_image.data;

  int curr_residual = 0;

  for (int r = 5; r < left_sdf_image.rows - 5; ++r){
    for (int c = 5; c < left_sdf_image.cols - 5; ++c){
      int i = r*left_classification_image.cols + c;

      if (curr_residual >= NUM_RESIDUALS) break;

      if (sdf_im_data[i] <= float(GLOBAL_ALS->GetHeavisideWidth()) - 1e-1 && sdf_im_data[i] >= -float(GLOBAL_ALS->GetHeavisideWidth()) + 1e-1){
        residuals[curr_residual] = GLOBAL_ALS->GetErrorValue(r, c, sdf_im_data[i], index_image_data[i]);
        double f = residuals[curr_residual];
        curr_residual++;
      }
    }
  }

  for (; curr_residual < NUM_RESIDUALS; ++curr_residual){
    residuals[curr_residual] = residuals[0];
  }

  if (jacobians != NULL) {

    cv::Matx<float, NUM_RESIDUALS, PRECISION> jacs = GLOBAL_ALS->ComputeJacobians(current_model_, left_classification_image, left_front_intersection, left_back_intersection, left_sdf_image, left_index_image);

    for (int res = 0; res < NUM_RESIDUALS; ++res){

      size_t current_parameter_idx = 0;

      ci::app::console() << "For residual " << res << " jacobian = \n[";

      for (int i = 0; i < parameter_block_sizes().size(); ++i){

        const int num_params_in_block = parameter_block_sizes()[i];

        if (jacobians[i] != NULL) {

          for (int param = 0; param < num_params_in_block; ++param){

            jacobians[i][res * num_params_in_block + param] = jacs(res, current_parameter_idx + param);
            if (jacobians[i][res * num_params_in_block + param])
              ci::app::console() << jacobians[i][res * num_params_in_block + param] << ", ";

          }

          current_parameter_idx += num_params_in_block;

        }
      }

      
      ci::app::console() << "]\n\n\n" << std::endl;
     
    }

  }

  return true;

}

void ArticulatedLevelSetSolver::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){


  frame_ = frame;
  current_model_ = current_model;

  std::vector<float> pose_parameters;
  current_model->GetPose(pose_parameters);

  //initalise
  double **parameters = new double*[3];
  parameters[0] = new double[3];
  parameters[1] = new double[4];
  parameters[2] = new double[5];

  parameters[0][0] = pose_parameters[0];
  parameters[0][1] = pose_parameters[1];
  parameters[0][2] = pose_parameters[2];

  parameters[1][0] = pose_parameters[3];
  parameters[1][1] = pose_parameters[4];
  parameters[1][2] = pose_parameters[5];
  parameters[1][3] = pose_parameters[6];

  parameters[2][0] = pose_parameters[7];
  parameters[2][1] = pose_parameters[8];
  parameters[2][2] = pose_parameters[9];
  parameters[2][3] = pose_parameters[10];
  parameters[2][4] = pose_parameters[11];

  ci::app::console() << "########### POSE PARAMETERS ################\n\n ";

  for (int i = 0; i < pose_parameters.size(); ++i){

    ci::app::console() << pose_parameters[i] << ", ";

  }

  ci::app::console() << "\n\======================================" << std::endl;;

  GLOBAL_ALS = new ArticulatedLevelSet(stereo_camera_);

  ceres::Problem problem;

  //option 1 use this version with anyltic derivatives
  //supply initial problem guess - parameters
  //problem.AddResidualBlock(this, nullptr, parameters); 

  //option 2 use automatic derivatives
   ceres::CostFunction* cost_function = new ArticulatedLevelSetSolver(stereo_camera_, current_model_, frame_);

  problem.AddResidualBlock(cost_function, nullptr, parameters[0], parameters[1], parameters[2]);
  problem.AddParameterBlock(parameters[0], 3);
  problem.AddParameterBlock(parameters[1], 4, new ceres::QuaternionParameterization());
  problem.AddParameterBlock(parameters[2], 5);
  
  //option 3 add each residual as a separate cost function
  //to do this one need to iterate over sdf image and get indexes of all pixels we want to process
  //also need to figure out how we can 'refresh' the model at each loop
  //for (size_t i = 0; i < row_col_residual_idx_.size(); ++i){
  //  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ArticulatedLevelSetSolverSingleResidual, 1, 3, 4, 1, 1, 1, 1, 1>(new ArticulatedLevelSetSolverSingleResidual(row_col_residual_idx_[i].first, row_col_residual_idx_[i].second));
  //  problem.AddResidualBlock(cost_function, nullptr, parameters);
  //}

 
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.update_state_every_iteration = true;
  //options.check_gradients = true;
  //options.jacobi_scaling = true;
  //options.minimizer_type = ceres::TRUST_REGION;
  //options.minimizer_type = ceres::LINE_SEARCH;
  //options.line_search_direction_type = ceres::STEEPEST_DESCENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  ci::app::console() << summary.FullReport() << std::endl;


  delete  GLOBAL_ALS;
  delete[] parameters[0];
  delete[] parameters[1];
  delete[] parameters[2];
  delete[] parameters;

}

void ArticulatedLevelSet::SetParameters(boost::shared_ptr<Model> current_model, double const* const*parameters){

  std::vector<float> f_params;
  //for (int i = 0; i < PRECISION; ++i){

  //  f_params.push_back(parameters[i]);

  //}
  for (int i = 0; i < 3; ++i){
    f_params.push_back(parameters[0][i]);
  }

  auto v = current_model->GetBasePose().GetPose();

  for (int i = 0; i < 4; ++i){
    f_params.push_back(v[3 + i]);
  }
  for (int i = 0; i < 5; ++i){
    f_params.push_back(parameters[2][i]);
  }

  current_model->SetPose(f_params);

}

cv::Matx<float, NUM_RESIDUALS, PRECISION> ArticulatedLevelSet::ComputeJacobians(const boost::shared_ptr<Model> current_model, const cv::Mat &classification_image, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image, const cv::Mat &sdf_image, const cv::Mat &index_image){

  cv::Matx<float, NUM_RESIDUALS, PRECISION > jacobians = cv::Matx<float, NUM_RESIDUALS, PRECISION>::zeros();

  float *sdf_im_data = (float *)sdf_image.data;
  float *front_intersection_data = (float *)front_intersection_image.data;
  float *back_intersection_data = (float *)back_intersection_image.data;
  unsigned char *index_image_data = (unsigned char *)index_image.data;

  //float error_value = 0.0f;

  int current_row = 0;

  for (int r = 5; r < classification_image.rows - 5; ++r){
    for (int c = 5; c < classification_image.cols - 5; ++c){

      int i = r*classification_image.cols + c;

      if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

        const float cost_value = GetErrorValue(r, c, sdf_im_data[i], index_image_data[i]);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const float region_agreement = GetRegionAgreement(r, c, sdf_im_data[i], index_image_data[i]);

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

        const float dsdf_dx = 0.5*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
        const float dsdf_dy = 0.5*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

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
      jacobians(current_row, a) = jacobians(0,a);
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
        const float region_agreement = GetRegionAgreement(r, c, sdf_im_data[i], index_image_data[i]);

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

void ArticulatedLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  if (curr_step == NUM_STEPS) curr_step = 0;

  ++curr_step;

  frame_ = frame;

  //index images give the index from the 
  cv::Mat left_sdf_image, left_front_intersection, left_back_intersection, left_index_image;
  cv::Mat right_sdf_image, right_front_intersection, right_back_intersection, right_index_image;
 
  const bool wp = true;
  const bool wy = true;
  const bool cp = true;

  //iterate until steps or convergences
  for (size_t step = 0; step < 1; ++step){
   
    //for prototyping the articulated jacs, we use a vector. this will be flattened for faster estimation later
    cv::Matx<float, 7, 1> j_sum_rigid = cv::Matx<float, 7, 1>::zeros();
    cv::Matx<float, 7, 7> h_sum_rigid = cv::Matx<float, 7, 7>::zeros();

    cv::Matx<float, 3, 1> j_sum_arti = cv::Matx<float, 3, 1>::zeros();

    const int max_pose_param = 4; // 0 = do rigid, 1 = wp, 2 = wy, 3 = cr

    if (current_pose_param_ >= max_pose_param) current_pose_param_ = 0;

    if (current_rigid_step_ >= num_rigid_steps_) {
      current_rigid_step_ = 0;
      current_pose_param_++;
    }

    ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->left_eye(), left_sdf_image, left_front_intersection, left_back_intersection, left_index_image);
    //ProcessArticulatedSDFAndIntersectionImage(current_model, stereo_camera_->right_eye(), right_sdf_image, right_front_intersection, right_back_intersection, right_index_image);

    cv::Mat &classification_image = frame_->GetClassificationMapROI();
    cv::Mat &front_intersection_image = left_front_intersection;
    cv::Mat &back_intersection_image = left_back_intersection;
    cv::Mat &sdf_image = left_sdf_image;
    cv::Mat &index_image = left_index_image;
    cv::Mat error_image = cv::Mat::zeros(sdf_image.size(), CV_32FC1);

    float *sdf_im_data = (float *)sdf_image.data;
    float *front_intersection_data = (float *)front_intersection_image.data;
    float *back_intersection_data = (float *)back_intersection_image.data;
    unsigned char *index_image_data = (unsigned char *)index_image.data;

    static std::vector<float> error_values;
    float error_value = 0.0f;

    for (int r = 5; r < classification_image.rows - 5; ++r){
      for (int c = 5; c < classification_image.cols - 5; ++c){

        int i = r*classification_image.cols + c;

        if (sdf_im_data[i] <= float(HEAVYSIDE_WIDTH) - 1e-1 && sdf_im_data[i] >= -float(HEAVYSIDE_WIDTH) + 1e-1){

          //-log(H * P_f + (1-H) * P_b)
          error_value += GetErrorValue(r, c, sdf_im_data[i], index_image_data[i]);
          error_image.at<float>(r, c) = error_value;
          

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const float region_agreement = GetRegionAgreement(r, c, sdf_im_data[i], index_image_data[i]);

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

          const float dsdf_dx = 0.5*(sdf_im_data[r*classification_image.cols + (c + 1)] - sdf_im_data[r*classification_image.cols + (c - 1)]);
          const float dsdf_dy = 0.5*(sdf_im_data[(r + 1)*classification_image.cols + c] - sdf_im_data[(r - 1)*classification_image.cols + c]);

          //update the jacobian
          UpdateArticulatedJacobian(region_agreement, index_image_data[shifted_i], sdf_im_data[i], dsdf_dx, dsdf_dy, stereo_camera_->left_eye()->Fx(), stereo_camera_->left_eye()->Fy(), front_intersection_image.at<cv::Vec3f>(shifted_i), back_intersection_image.at<cv::Vec3f>(shifted_i), current_model, jacs);

          cv::Matx<float, 1, 7> rigid_jacs;
          
          //copy the rigid
          for (int j = 0; j < 7; ++j){
            rigid_jacs(j) = jacs(j);
          }
         
          j_sum_arti(0) += jacs(7);
          j_sum_arti(1) += jacs(8);
          j_sum_arti(2) += jacs(9) + jacs(10); //jacs(10) is the 'wrong' way so it's value must be negative


          j_sum_rigid += rigid_jacs.t();
          h_sum_rigid += (rigid_jacs.t() * rigid_jacs);
        }
      }
    }

    UpdateWithErrorValue(error_value);

    if (previous_error_value_ < 0) {
      previous_error_value_ = error_value;
    }
    else{
      if (error_value > previous_error_value_ && current_pose_param_ == 0){
        current_pose_param_++;
      }
    }

    j_sum_rigid = h_sum_rigid.inv() * j_sum_rigid;

    std::vector<float> jacs(PRECISION, 0);
    
    if (current_pose_param_ == 0){
      for (size_t v = 0; v < 7; ++v){
        jacs[v] = -j_sum_rigid(v);
      }
    }

    
    if (current_pose_param_ == 1 && wp){
      jacs[7] = -(j_sum_arti(0) / (100 * std::abs(j_sum_arti(0))));
    }

    if (current_pose_param_ == 2 && wy){
      jacs[8] = -(j_sum_arti(1) / (100 * std::abs(j_sum_arti(1))));

    }

    if (current_pose_param_ == 3 && cp){

      jacs[9] = -(j_sum_arti(2) / (100 * std::abs(j_sum_arti(2))));

      jacs[10] = (j_sum_arti(2) / (100 * std::abs(j_sum_arti(2)))); //should be 'negative'
    }

    
    ci::app::console() << "Jacobian = [";
    for (size_t v = 0; v < jacs.size(); ++v){
      ci::app::console() << jacs[v] << ",";
    }
    ci::app::console() << "]" << std::endl;

    ////jacs[jacs.size() - 1] = -jacs[jacs.size() - 2];

    if (current_pose_param_ == 0){
      current_rigid_step_++;
    }
    else{
      current_pose_param_++;
    }

    current_model->UpdatePose(jacs);

  }//end for

  //for each pixel find the sdf that it corrsponds to 

  //each component can have a colour model (one against all classification)

  //region agreement is computed as same

  //3d intersection values need to be in camera frame and LOCAL coordinate frame of the geometry

  //update the correct part of the jacobian with the derivative

  //shaft -> rigid se3 
  //head -> wrist pitch
  //jointly solve claspers and wrist yaw

}

float ArticulatedLevelSet::GetErrorValue(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const{

  const float pixel_probability = frame_->GetClassificationMapROI().at<float>(row_idx, col_idx);
  assert(pixel_probability >= 0.0f && pixel_probability <= 1.0f);

  const float heaviside_value = HeavisideFunction(sdf_value);

  float v = (heaviside_value * pixel_probability) + ((1 - heaviside_value)*(1 - pixel_probability));
  v += 0.0000001;
  return -log(v);

}

float ArticulatedLevelSet::GetRegionAgreement(const int row_idx, const int col_idx, const float sdf_value, const int target_label) const{

  const float pixel_probability = frame_->GetClassificationMapROI().at<float>(row_idx, col_idx);
  const float heaviside_value = HeavisideFunction(sdf_value);
  const float heaviside_2 = 0.5*(1.0 + sdf_value / float(HEAVYSIDE_WIDTH) + (1.0 / M_PI)*sin((M_PI*sdf_value) / float(HEAVYSIDE_WIDTH)));
  const float epsilon = 1e-10;
  const float Pf = pixel_probability;
  const float Pb = (1 - pixel_probability);

  float val = (Pf - Pb) / (Pf * heaviside_value + Pb * (1 - heaviside_value) + epsilon);
  return (2.0f*pixel_probability - 1.0f) / (heaviside_value*pixel_probability + (1.0f - heaviside_value)*(1.0f - pixel_probability));

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

  ProcessSDFAndIntersectionImage(mesh, stereo_camera_->left_eye(), composite_sdf_image, composite_front_intersection_image, composite_back_intersection_image);
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
    ProcessSDFAndIntersectionImage(mesh, stereo_camera_->left_eye(), sdf_image, front_intersection_image, back_intersection_image);

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

