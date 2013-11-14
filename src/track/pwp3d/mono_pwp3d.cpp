#include "../../../headers/track/pwp3d/mono_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include<boost/filesystem.hpp>
using namespace ttrk;


Pose MonoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;
  SetBlurringScaleFactor(frame_->GetImageROI().cols);
  const int NUM_STEPS = 200;
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
  boost::filesystem::create_directory(ss.str()+"/mono");
  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat canvas = frame_->GetImageROI().clone();
  DrawModelOnFrame(current_model.ModelPointsAtCurrentPose(),canvas);
  cv::imwrite(ss.str()+"/mono/init.png",canvas);
  cv::imwrite(ss.str()+"/mono/classification.png",frame_->GetClassificationMapROI());         
  frame_count++;

  DEBUG_DIR_ = ss.str() + "/debug/";
  boost::filesystem::create_directory(DEBUG_DIR_);
  std::ofstream ENERGY_FILE((DEBUG_DIR_ + "/energy_file.csv").c_str());
  if(!ENERGY_FILE.is_open()) throw(std::runtime_error("Error, could not open energy file!\n"));
#endif


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

#ifdef SAVEDEBUG_1

    std::stringstream step_dir; step_dir << "step" << step;
    boost::filesystem::create_directory(ss.str()+"/mono/"+step_dir.str());

#endif

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    double energy = 0.0;
    double pixel_count = 0;

    cv::Mat sdf_image,front_intersection_image,back_intersection_image;
    GetSDFAndIntersectionImage(current_model,sdf_image,front_intersection_image,back_intersection_image);

#ifdef SAVEDEBUG_2
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/sdf.png",sdf_image);
    cv::Mat canvas = frame_->GetImageROI().clone();
    DrawModelOnFrame(current_model.ModelPointsAtCurrentPose(),canvas);
    cv::imwrite(ss.str()+"/mono/"+step_dir.str()+"/previous.png",canvas);
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

    for(int r=0;r<frame_->GetImageROI().rows;r++){
      for(int c=0;c<frame_->GetImageROI().cols;c++){


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
        pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model, front_intersection_image, back_intersection_image);

        //update the jacobian
        for(int i=0;i<pose_derivatives.rows;i++){
          double pp = (region_agreement*pose_derivatives.at<double>(i,0));
          if (pp != pp) { std::cerr << "Alert! Bad derivatives\n"; continue; }
          jacobian.at<double>(i,0) += -1 * pp * 5;
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

#ifdef SAVEDEBUG_2
    cv::Mat heaviside(jacobian_x.size(),CV_8UC1);
    cv::Mat delta(jacobian_x.size(),CV_8UC1);
    cv::Mat dSDFdx_save(jacobian_x.size(),CV_8UC3);
    cv::Mat dSDFdy_save(jacobian_x.size(),CV_8UC3);
    for(int r=0;r<heaviside.rows;r++){
      for(int c=0;c<heaviside.cols;c++){
        heaviside.at<unsigned char>(r,c) = 255 * Heaviside(sdf_image.at<float>(r,c), k_heaviside_width_ * blurring_scale_factor_);
        delta.at<unsigned char>(r,c) = 255 * DeltaFunction(sdf_image.at<float>(r,c),k_delta_function_std_ * blurring_scale_factor_);
        dSDFdx_save.at<cv::Vec3b>(r,c) = cv::Vec3b((255 * (dSDFdx.at<float>(r,c) > 0)) * dSDFdx.at<float>(r,c),0,(255 * (dSDFdx.at<float>(r,c) < 0)) * -dSDFdx.at<float>(r,c));
        dSDFdy_save.at<cv::Vec3b>(r,c) = cv::Vec3b((255 * (dSDFdy.at<float>(r,c) > 0)) * dSDFdy.at<float>(r,c),0,(255 * (dSDFdy.at<float>(r,c) < 0)) * -dSDFdy.at<float>(r,c));
      }
    }

    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/heaviside.png",heaviside);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/delta.png",delta);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/jacobian_x.png",jacobian_x);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/jacobian_y.png",jacobian_y);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/jacobian_z.png",jacobian_z);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/energy.png",energy_image);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/dsf_dx.png",dSDFdx_save);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/dsf_dy.png",dSDFdy_save);
    cv::imwrite(ss.str() + "/mono/" + step_dir.str() + "/region_agreement.png",region_agreement_image);

    ENERGY_FILE << energy << " ";
    ENERGY_FILE.flush();
    //std::cout << "ENERGY IS : " << energy << std::endl;
#endif



    energy_vals.push_back(energy);


    std::vector<MatchedPair> pnp_pairs;
    register_points_.FindPointCorrespondencesWithPose(frame_,pnp_pairs,current_model.CurrentPose());
    #ifdef SAVEDEBUG_2
    std::cerr << "Found " << pnp_pairs.size() << " matches!\n";
    #endif

    for(auto pnp=pnp_pairs.begin();pnp!=pnp_pairs.end();pnp++){
      cv::Mat pnp_jacobian = register_points_.GetPointDerivative(pnp->learned_point,cv::Point2f(pnp->image_point[0],pnp->image_point[1]), current_model.CurrentPose());
      for(int i=0;i<jacobian.rows;i++){
        if(i < 3)
          jacobian.at<double>(i,0) += 5 * -1 * pnp_jacobian.at<double>(i,0);
        else
          jacobian.at<double>(i,0) += 5 * pnp_jacobian.at<double>(i,0);
      }
    }

    ApplyGradientDescentStep(jacobian,current_model.CurrentPose(),step,pixel_count);

    //save the step estimate
#ifdef SAVEDEBUG_1
    canvas = frame_->GetImageROI().clone();
    DrawModelOnFrame(current_model.ModelPointsAtCurrentPose(),canvas);
    cv::imwrite(ss.str()+"/mono/"+step_dir.str()+"/update.png",canvas);
    cv::imwrite(ss.str()+"/mono/"+step_dir.str()+".png",canvas);
#endif

    //test for convergence
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose() );
    //converged = HasGradientDescentConverged__new(std::vector<cv::Mat>(), jacobian );
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose());
    //if(!first)
    //  converged = HasGradientDescentConverged_UsingEnergy(energy_vals);

    //if(energy>max_energy){
    {
      //std::cerr << "new max energy = " << energy << "\n";
      max_energy = energy;
      pwp3d_best_pose = current_model.CurrentPose();
#ifdef SAVEDEBUG_2
      best_image = canvas.clone();
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


