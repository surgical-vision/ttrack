#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include<boost/filesystem.hpp>


using namespace ttrk;
/*** REMOVE THIS ***/


bool StereoPWP3D::SetupEye(const int eye, Pose &pose){

  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  if(eye == 0) {

    //do nothing for the first left eye
    return true;

  }else if(eye == 1){

    Camera() = GetStereoCamera()->rectified_right_eye();

    //swap the roi over in the images so they refer to the right hand image
    stereo_frame->SwapEyes();
    //also update the object pose so that it's relative to the right eye
    Pose extrinsic;
    if(stereo_camera_->IsRectified())
      extrinsic = Pose(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    else
      extrinsic = Pose(stereo_camera_->ExtrinsicTranslation(),sv::Quaternion(stereo_camera_->ExtrinsicRotation()));
    
    pose = CombinePoses(extrinsic, pose);

    return true;

  }else{

    Camera() = GetStereoCamera()->rectified_left_eye();

    //swap everything back
    stereo_frame->SwapEyes();

    Pose extrinsic;
    if(stereo_camera_->IsRectified())
      extrinsic = Pose(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    else
      extrinsic = Pose(stereo_camera_->ExtrinsicTranslation(),sv::Quaternion(stereo_camera_->ExtrinsicRotation()));

    pose = CombinePoses(extrinsic.Inverse(),pose);

    return false;

  }

}

cv::Mat StereoPWP3D::GetPoseDerivativesRightEye(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model, const cv::Mat &front_intersection_image, const cv::Mat &back_intersection_image){

  const int NUM_DERIVS = 7;
     
   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model, front_intersection_image, back_intersection_image);
  
  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,current_model, front_intersection_image, back_intersection_image);
    if(!intersects)
      //throw(std::runtime_error("Error, should not miss point on the border. Check GetNearestIntesection::search_range and continue values!\n"));
      return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  }


  //just in case...
  if(front_intersection[2] == 0) front_intersection[2] = 0.00000001;
  const double z_inv_sq_front = 1.0/(front_intersection[2]*front_intersection[2]);
  if(back_intersection[2] == 0) back_intersection[2] = 0.00000001;
  const double z_inv_sq_back = 1.0/(back_intersection[2]*back_intersection[2]);

  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  for(int dof=0;dof<NUM_DERIVS;dof++){

    //compute the derivative for each dof
    const cv::Vec3f dof_derivatives_front = GetDOFDerivativesRightEye(dof,current_model.CurrentPose(),front_intersection);
    const cv::Vec3f dof_derivatives_back = GetDOFDerivativesRightEye(dof,current_model.CurrentPose(),back_intersection);

    const double dXdL = camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2])));
    ret.at<double>(dof,0) = DeltaFunction(sdf.at<float>(r,c), blurring_scale_factor_ * k_delta_function_std_ ) * ((dSDFdx * dXdL) + (dSDFdy * dYdL));
      
  }
  return ret;

}


cv::Vec3f StereoPWP3D::GetDOFDerivativesRightEye(const int dof, const Pose &pose, const cv::Vec3f &point_) {

  //derivatives use the (x,y,z) from the initial reference frame not the transformed one so inverse the transformation
  
  Pose extrinsic;
  if(stereo_camera_->IsRectified())
    extrinsic = Pose(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
  else
    extrinsic = Pose(stereo_camera_->ExtrinsicTranslation(),sv::Quaternion(stereo_camera_->ExtrinsicRotation()));


  const cv::Vec3f point = extrinsic.InverseTransform(point_); //point in left camera coordinates

  cv::Vec3f derivs = CombinePoses(extrinsic.Inverse(),pose).GetDOFDerivatives(dof,point); //get derivatives from the perspective of the left camera

  return extrinsic.rotation_.RotateVector(derivs);

}

void StereoPWP3D::DrawModelOnBothFrames(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas) {

  Camera() = GetStereoCamera()->rectified_left_eye();
  DrawModelOnFrame(tracked_model.ModelPointsAtCurrentPose(),left_canvas);
  
  KalmanTracker tracked_model_from_right = tracked_model;
  Pose extrinsic;
  if(stereo_camera_->IsRectified())
    extrinsic = Pose(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
  else
    extrinsic = Pose(stereo_camera_->ExtrinsicTranslation(),sv::Quaternion(stereo_camera_->ExtrinsicRotation()));
  
  Camera() = GetStereoCamera()->rectified_right_eye();
  tracked_model_from_right.CurrentPose() = CombinePoses(extrinsic, tracked_model_from_right.CurrentPose());
  DrawModelOnFrame(tracked_model_from_right.ModelPointsAtCurrentPose(),right_canvas);
  Camera() = GetStereoCamera()->rectified_left_eye();

}

Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){
 
  frame_ = frame;
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
  SetBlurringScaleFactor(stereo_frame->GetLeftImage().cols);
  const int NUM_STEPS = 15;
  cv::Vec3f initial_translation = current_model.CurrentPose().translation_;
  static bool first = true;

#ifdef DEBUG
  boost::progress_timer t; //timer prints time when it goes out of scope
#endif


#ifdef SAVEDEBUG_1
  static int frame_count = 0;
  boost::filesystem::create_directory("debug");
  std::stringstream ss; ss << "debug/frame_" << frame_count;
  boost::filesystem::create_directory(ss.str());
  boost::filesystem::create_directory(ss.str()+"/left");
  boost::filesystem::create_directory(ss.str()+"/right");
  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
  cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
  DrawModelOnBothFrames(current_model,left_canvas,right_canvas);
  
  cv::imwrite(ss.str()+"/left/init.png",left_canvas);
  cv::imwrite(ss.str()+"/right/init.png",right_canvas);  
  cv::imwrite(ss.str()+"/left/classification.png",stereo_frame->GetLeftClassificationMap());
  cv::imwrite(ss.str()+"/right/classification.png",stereo_frame->GetRightClassificationMap());
  frame_count++;
  
  DEBUG_DIR_ = ss.str() + "/debug/";
  boost::filesystem::create_directory(DEBUG_DIR_);
  std::ofstream ENERGY_FILE((DEBUG_DIR_ + "/energy_file.csv").c_str());
  if(!ENERGY_FILE.is_open()) throw(std::runtime_error("Error, could not open energy file!\n"));
#endif

  /*cv::Mat sdf_image,front_intersection_image,back_intersection_image;
  GetSDFAndIntersectionImage(current_model,sdf_image,front_intersection_image,back_intersection_image);
  register_points_.ComputeDescriptorsForPointTracking(frame_,current_model,sdf_image);
  SAFE_EXIT();*/

  //store a vector of Pose values. check std. dev. of these values, if this is small enough, assume convergence.
  //std::vector<double> convergence_test_values;
  std::deque<Pose> convergence_test_values;
  bool converged = false;

  //values to hold the 'best' pwp3d estimate
  double max_energy = 0;
  Pose pwp3d_best_pose = current_model.CurrentPose();
  cv::Mat best_image;
  std::vector<double> energy_vals;

  //iterate until convergence
  for(int step=0; step < NUM_STEPS && !converged; step++){

#if defined(SAVDEBUG_1) || defined(SAVEDEBUG_2)
    std::stringstream step_dir; step_dir << "step" << step;
#endif
#ifdef SAVEDEBUG_2


    boost::filesystem::create_directory(ss.str()+"/left/"+step_dir.str());
    boost::filesystem::create_directory(ss.str()+"/right/"+step_dir.str());

#endif

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    double energy = 0.0;
    double pixel_count = 0;

    for(int eye=0; ;eye++){

      if(!SetupEye(eye,current_model.CurrentPose())) break; //sets the camera_ variable to left or right eye breaks after resetting everything back to initial state after right ey

      cv::Mat sdf_image,front_intersection_image,back_intersection_image;
      GetSDFAndIntersectionImage(current_model,sdf_image,front_intersection_image,back_intersection_image);

#ifdef SAVEDEBUG_2
      if(eye == 0){
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/sdf.png",sdf_image);
        cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
        DrawModelOnFrame(current_model.ModelPointsAtCurrentPose(),left_canvas);
        cv::imwrite(ss.str()+"/left/"+step_dir.str()+"/previous.png",left_canvas);
      }
      if(eye == 1) {
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/sdf.png",sdf_image);
        cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
        DrawModelOnFrame(current_model.ModelPointsAtCurrentPose(),right_canvas);
        cv::imwrite(ss.str()+"/right/"+step_dir.str()+"/previous.png",right_canvas);
      }
#endif

      //compute the derivates of the sdf images
      cv::Mat dSDFdx, dSDFdy;
      
      //cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1); // (src,dst,dtype,dx,dy,size) size = 1 ==> 3x1 finite difference kernel
      //cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);
      cv::Scharr(sdf_image,dSDFdx,CV_32FC1,1,0);
      cv::Scharr(sdf_image,dSDFdy,CV_32FC1,0,1);

#ifdef SAVEDEBUG_2
      cv::Mat jacobian_x = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat jacobian_y = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat jacobian_z = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat energy_image = cv::Mat::zeros(sdf_image.size(),CV_8UC1);
      cv::Mat region_agreement_image = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
#endif

      //std::cerr << "\n\nCurrently subsampling frame!\n\n";

      for(int r=0;r<frame_->GetImageROI().rows;r+=3){
        for(int c=0;c<frame_->GetImageROI().cols;c+=3){

          //speedup tests by checking if we need to evaluate the cost function in this region
          const double skip = Heaviside(sdf_image.at<float>(r,c), k_heaviside_width_ * blurring_scale_factor_);
          if( skip < 0.00001 || skip > 0.99999 ) continue;

          //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/          
          energy += GetEnergy(r,c,sdf_image.at<float>(r,c)); 
#ifdef SAVEDEBUG_2
          energy_image.at<unsigned char>(r,c) = 255 * GetEnergy(r,c,sdf_image.at<float>(r,c));
#endif
          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c));

          //dH / dL
          cv::Mat pose_derivatives;
          if(eye == 0)
            pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model, front_intersection_image, back_intersection_image);
          else if(eye == 1)
            pose_derivatives = GetPoseDerivativesRightEye(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model, front_intersection_image, back_intersection_image);
          else
            throw(std::runtime_error("Error, bad stereo thing.\n"));

          //update the jacobian
          for(int i=0;i<pose_derivatives.rows;i++){
            double pp = (region_agreement*pose_derivatives.at<double>(i,0));
            if (pp != pp) { 
              std::cerr << "Alert! Bad derivatives\n"; 
              continue; 
            }
            jacobian.at<double>(i,0) += -1 * pp;
          }
          

          pixel_count++ ;
          
#ifdef SAVEDEBUG_2

        if(region_agreement > 0)
          region_agreement_image.at<cv::Vec3b>(r,c) = cv::Vec3b(255,0,0);
        else if(region_agreement < 0)
          region_agreement_image.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255);

        double max_val = std::max(std::abs(pose_derivatives.at<double>(0,0)),std::abs(pose_derivatives.at<double>(1,0)));
        max_val = std::max(max_val,std::abs(pose_derivatives.at<double>(2,0)));

        if(region_agreement*pose_derivatives.at<double>(0,0) > 0)
          jacobian_x.at<cv::Vec3b>(r,c) = cv::Vec3b(255.0*pose_derivatives.at<double>(0,0)/max_val,0,0); //blue right
        else if(region_agreement*pose_derivatives.at<double>(0,0) < 0)
          jacobian_x.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255.0*pose_derivatives.at<double>(0,0)/max_val); //red left

        if(region_agreement*pose_derivatives.at<double>(1,0) > 0)
          jacobian_y.at<cv::Vec3b>(r,c) = cv::Vec3b(255.0*pose_derivatives.at<double>(1,0)/max_val,0,0); //blue down
        else if (region_agreement*pose_derivatives.at<double>(1,0) < 0)
          jacobian_y.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255.0*pose_derivatives.at<double>(1,0)/max_val); //red up

        if(region_agreement*pose_derivatives.at<double>(2,0) > 0)
          jacobian_z.at<cv::Vec3b>(r,c) = cv::Vec3b(255.0*pose_derivatives.at<double>(2,0)/max_val,0,0); //blue away
        else if(region_agreement*pose_derivatives.at<double>(2,0) < 0)
          jacobian_z.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255.0*pose_derivatives.at<double>(2,0)/max_val); //red towards

#endif
        }
      }

    cv::Mat save_edge_image = frame_->GetImageROI().clone();
    cv::Mat edge_jacobian = AlignObjectToEdges(current_model,frame_->GetClassificationMapROI() , sdf_image, front_intersection_image,save_edge_image );
#ifdef SAVEDEBUG_2
    if(eye == 0)
      cv::imwrite(ss.str()+"/left/"+step_dir.str()+"/edges.png",save_edge_image);
    else
      cv::imwrite(ss.str()+"/right/"+step_dir.str()+"/edges.png",save_edge_image);
#endif

    for(int i=0;i<jacobian.rows;i++){
        jacobian.at<double>(i,0) += -20 * edge_jacobian.at<double>(i,0);
    }

#ifdef SAVEDEBUG_2
      cv::Mat heaviside(jacobian_x.size(),CV_8UC1);
      cv::Mat delta(jacobian_x.size(),CV_8UC1);
      cv::Mat dSDFdx_save(jacobian_x.size(),CV_8UC3);
      cv::Mat dSDFdy_save(jacobian_x.size(),CV_8UC3);
      for(int r=0;r<heaviside.rows;r++){
        for(int c=0;c<heaviside.cols;c++){
          heaviside.at<unsigned char>(r,c) = 255 * Heaviside(sdf_image.at<float>(r,c), k_heaviside_width_ * blurring_scale_factor_ );
          delta.at<unsigned char>(r,c) = 255 * DeltaFunction(sdf_image.at<float>(r,c), k_delta_function_std_ * blurring_scale_factor_ );
          dSDFdx_save.at<cv::Vec3b>(r,c) = cv::Vec3b((255 * (dSDFdx.at<float>(r,c) > 0)) * dSDFdx.at<float>(r,c),0,(255 * (dSDFdx.at<float>(r,c) < 0)) * -dSDFdx.at<float>(r,c));
          dSDFdy_save.at<cv::Vec3b>(r,c) = cv::Vec3b((255 * (dSDFdy.at<float>(r,c) > 0)) * dSDFdy.at<float>(r,c),0,(255 * (dSDFdy.at<float>(r,c) < 0)) * -dSDFdy.at<float>(r,c));
        }
      }

      if(eye == 0){
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/heaviside.png",heaviside);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/delta.png",delta);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/jacobian_x.png",jacobian_x);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/jacobian_y.png",jacobian_y);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/jacobian_z.png",jacobian_z);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/energy.png",energy_image);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/dsf_dx.png",dSDFdx_save);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/dsf_dy.png",dSDFdy_save);
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/region_agreement.png",region_agreement_image);
      }else if(eye == 1) {
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/heaviside.png",heaviside);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/delta.png",delta);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_x.png",jacobian_x);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_y.png",jacobian_y);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_z.png",jacobian_z);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/energy.png",energy_image);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/dsf_dx.png",dSDFdx_save);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/dsf_dy.png",dSDFdy_save);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/region_agreement.png",region_agreement_image);
      }



      ENERGY_FILE << energy << " ";
      ENERGY_FILE.flush();
      //std::cout << "ENERGY IS : " << energy << std::endl;
#endif

    }   

    energy_vals.push_back(energy);

    std::vector<MatchedPair> pnp_pairs;
    cv::Mat point_save_image = frame_->GetImageROI().clone();
    register_points_.FindPointCorrespondencesWithPose(frame_,pnp_pairs,current_model.CurrentPose(),point_save_image);
#ifdef SAVEDEBUG_2
    cv::imwrite(ss.str()+"/left/"+step_dir.str()+"/points.png",point_save_image);
    std::cerr << "Found " << pnp_pairs.size() << " matches!\n";
#endif
    for(auto pnp=pnp_pairs.begin();pnp!=pnp_pairs.end();pnp++){
      cv::Mat pnp_jacobian = register_points_.GetPointDerivative(pnp->learned_point,cv::Point2f(pnp->image_point[0],pnp->image_point[1]), current_model.CurrentPose());
      for(int i=0;i<jacobian.rows;i++){
        jacobian.at<double>(i,0) += 2 * -1 * pnp_jacobian.at<double>(i,0);
      }
    }

    ApplyGradientDescentStep(jacobian,current_model.CurrentPose(),step,pixel_count);

    //save the step estimate
#ifdef SAVEDEBUG_1
    cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
    cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
    DrawModelOnBothFrames(current_model,left_canvas,right_canvas);
    cv::imwrite(ss.str()+"/left/"+step_dir.str()+".png",left_canvas);
    cv::imwrite(ss.str()+"/right/"+step_dir.str()+".png",right_canvas);
#endif

    //test for convergence
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose() );
    //converged = HasGradientDescentConverged__new(std::vector<cv::Mat>(), jacobian );
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose());
    if(!first)
      converged = HasGradientDescentConverged_UsingEnergy(energy_vals);

    //if(energy>max_energy){
    {
      //std::cerr << "new max energy = " << energy << "\n";
      max_energy = energy;
      pwp3d_best_pose = current_model.CurrentPose();
#ifdef SAVEDEBUG_2
      best_image = left_canvas.clone();
#endif
    }

  }

#ifdef SAVEDEBUG_1
  ENERGY_FILE.close();
#endif

  current_model.CurrentPose() = pwp3d_best_pose;

  //update the velocity model... a bit crude
  cv::Vec3f translational_velocity = current_model.CurrentPose().translation_ - initial_translation;
  current_model.CurrentPose().translational_velocity_ = translational_velocity;
  first = false;
  return current_model.CurrentPose();

}


bool StereoPWP3D::HasGradientDescentConverged__new(std::vector<cv::Mat> &convergence_test_values, cv::Mat &current_estimate) const {

  convergence_test_values.push_back(current_estimate);

  double sum_jacobian = 0;
  for(int i=0;i<current_estimate.rows;i++){
    sum_jacobian += std::abs(current_estimate.at<double>(i,0));
  }

  return sum_jacobian < 9e8;

}

bool StereoPWP3D::HasGradientDescentConverged(std::deque<Pose> &previous_poses, const Pose &pose) const {

  const int NUM_VALUES_TO_USE = 10;

  previous_poses.push_back(pose);
  if(previous_poses.size() < NUM_VALUES_TO_USE) return false;
  previous_poses.pop_front();

  cv::Mat sum = cv::Mat::zeros(7,1,CV_64FC1);
  cv::Mat sum_sqrs = cv::Mat::zeros(7,1,CV_64FC1);
  cv::Mat std_dev(7,1,CV_64FC1);
  for(auto val = previous_poses.begin(); val != previous_poses.end(); val++ ){

    int N = 0;
    sum.at<double>(N,0) += val->translation_[0];
    sum_sqrs.at<double>(N,0) += (val->translation_[0]*val->translation_[0]);
        
    N++;
    sum.at<double>(N,0) += val->translation_[1];
    sum_sqrs.at<double>(N,0) += (val->translation_[1]*val->translation_[1]);

    N++;
    sum.at<double>(N,0) += val->translation_[2];
    sum_sqrs.at<double>(N,0) += (val->translation_[2]*val->translation_[2]);

    N++;
    sum.at<double>(N,0) += val->rotation_.W();
    sum_sqrs.at<double>(N,0) += (val->rotation_.W()*val->rotation_.W());

    N++;
    sum.at<double>(N,0) += val->rotation_.X();
    sum_sqrs.at<double>(N,0) += (val->rotation_.X()*val->rotation_.X());

    N++;
    sum.at<double>(N,0) += val->rotation_.Y();
    sum_sqrs.at<double>(N,0) += (val->rotation_.Y()*val->rotation_.Y());

    N++;
    sum.at<double>(N,0) += val->rotation_.Z();
    sum_sqrs.at<double>(N,0) +=  (val->rotation_.Z()*val->rotation_.Z());

  }

  for(int i=0;i<std_dev.rows;i++){

    std_dev.at<double>(i,0) = sqrt((sum_sqrs.at<double>(i,0) - (sum.at<double>(i,0)*sum.at<double>(i,0))/previous_poses.size())/(previous_poses.size() - 1));
    

  }

  std::cerr << "Std Devs are " << std_dev << "\n";
  
  return false;
  
}

bool StereoPWP3D::HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const {

  std::cerr << "Energy for this step is " << energy_values.back() << "\n";
  const int NUM_VALUES_TO_USE = 7;
  if(energy_values.size() < NUM_VALUES_TO_USE ) return false;


  double energy_change = 0.0;
  for(auto value = energy_values.end()-(NUM_VALUES_TO_USE); value != energy_values.end()-1; value++ ){

    //std::cerr << "Energy change is: " << *(value+1) - *(value) << "\n";
    energy_change += *(value+1) - *(value);
    //std::cerr << "New energy is: " << energy_change << "\n";

  }

  energy_change /= NUM_VALUES_TO_USE - 1;

  std::cerr << "Current ratio is: " <<  energy_change/energy_values.back() << "\n";
  std::cerr << "Target ratio for convergence is: " << 1.0/1000 << "\n";

  return !(energy_change/energy_values.back() > 1.0/1000);

}
