#include <cinder/app/App.h>

#include "../../../include/track/articulated/articulated_solver.hpp"

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
  for (size_t i = 0; i < parameter_block_sizes().size(); ++i){

    int num_params_in_block = parameter_block_sizes()[i];

    for (size_t c = 0; c < num_params_in_block; ++c){

      ci::app::console() << parameters[i][c] << ", ";

    }

  }

  ci::app::console() << "]" << std::endl;

  GLOBAL_ALS->SetParameters(current_model_, parameters);

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

  for (size_t i = 0; i < pose_parameters.size(); ++i){

    ci::app::console() << pose_parameters[i] << ", ";

  }

  ci::app::console() << "\n\n======================================" << std::endl;;

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
