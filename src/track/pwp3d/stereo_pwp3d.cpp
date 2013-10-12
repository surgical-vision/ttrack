#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include<boost/filesystem.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <numeric>

using namespace ttrk;
//#define SAVEDEBUG

/*** REMOVE THIS ***/


void FindTransformationToImagePlane(std::vector<DescriptorMatches> matches,cv::Mat &rotation, cv::Mat &translation,  boost::shared_ptr<StereoCamera> cam, Pose current_pose);
void GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds);
void MatchDescriptorsToModel(std::vector<Descriptor> &d1, std::vector<Descriptor> &d2, std::vector<DescriptorMatches> &dm);
void ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count);

inline double l2_norm(const cv::Mat &a, const cv::Mat &b) {

  double ret = 0.0;
  if(a.size() != b.size()) throw(std::runtime_error("Error, a & b must have same dimensions in l2 norm!\n"));

  for(int r=0;r<a.rows;r++){
    for(int c=0;c<a.cols;c++){
      if(a.type() == CV_64FC1) ret += (a.at<double>(r,c) - b.at<double>(r,c))*(a.at<double>(r,c) - b.at<double>(r,c));
      else if(a.type() == CV_32FC1) ret += (a.at<float>(r,c) - b.at<float>(r,c))*(a.at<float>(r,c) - b.at<float>(r,c));
      else throw(std::runtime_error("Error, unsupported matrix type in l2 norm!\n"));
    }
  }

  return std::sqrt(ret);
}

const int NUM_DESCRIPTOR = 60;
const int MATCHING_DISTANCE_THRESHOLD = 40;
const double DESCRIPTOR_SIMILARITY_THRESHOLD = 200.0;

bool StereoPWP3D::SetupEye(const int eye, Pose &pose){

  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);

  if(eye == 0) {
    
    //do nothing for the first left eye
    return true;
  
  }else if(eye == 1){
    
    //swap the roi over in the images so they refer to the right hand image
    stereo_frame->SwapEyes();
    //also update the object pose so that it's relative to the right eye
    Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    pose = CombinePoses(extrinsic, pose);
 
    return true;

  }else{
    
    //swap everything back
    stereo_frame->SwapEyes();
    
    Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    pose = CombinePoses(extrinsic.Inverse(),pose);

    return false;

  }

}


void StereoPWP3D::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas) {

  {
    std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();

    for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

      cv::Vec2f projected = camera_->ProjectPoint(point->vertex_);

      for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){

        const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
        cv::Vec2f projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

        if(left_canvas.channels() == 3)
          line(left_canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),1,CV_AA);
        if(left_canvas.channels() == 1)
          line(left_canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
      }
    }
  }
  {
    KalmanTracker tracked_model_from_right = tracked_model;
    
    Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    tracked_model_from_right.CurrentPose() = CombinePoses(extrinsic, tracked_model_from_right.CurrentPose());
    std::vector<SimplePoint<> > transformed_points = tracked_model_from_right.ModelPointsAtCurrentPose();

    for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

      cv::Vec2f projected = GetStereoCamera()->rectified_right_eye()->ProjectPoint(point->vertex_);

      for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){

        const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
        cv::Vec2f projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

        if(right_canvas.channels() == 3)
          line(right_canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),1,CV_AA);
        if(right_canvas.channels() == 1)
          line(right_canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
      }
    }

  }

}

Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){
#ifdef SAVEDEBUG
//#undef SAVEDEBUG
#endif
  frame_ = frame;
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
  const int NUM_STEPS = 15;
  cv::Vec3f initial_translation = current_model.CurrentPose().translation_;

#ifdef DEBUG
    boost::progress_timer t; //timer prints time when it goes out of scope
#endif


#ifdef SAVEDEBUG
  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
  cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
  DrawModelOnFrame(current_model,left_canvas,right_canvas);
  static int frame_count = 0;
  boost::filesystem::create_directory("debug");
  std::stringstream ss; ss << "debug/frame_" << frame_count;
  boost::filesystem::create_directory(ss.str());
  boost::filesystem::create_directory(ss.str()+"/left");
  boost::filesystem::create_directory(ss.str()+"/right");
  cv::imwrite(ss.str()+"/left/init.png",left_canvas);
  cv::imwrite(ss.str()+"/right/init.png",right_canvas);
  frame_count++;
  
  DEBUG_DIR_ = ss.str() + "/debug/";
  boost::filesystem::create_directory(DEBUG_DIR_);
  std::ofstream ENERGY_FILE((DEBUG_DIR_ + "/energy_file.csv").c_str());
  if(!ENERGY_FILE.is_open()) throw(std::runtime_error("Error, could not open energy file!\n"));
#endif

  //ComputeDescriptorsForPointTracking(frame_,current_model);

  //store a vector of Pose values. check std. dev. of these values, if this is small enough, assume convergence.
  std::vector<cv::Mat> convergence_test_values;
  bool converged = false;

  //values to hold the 'best' pwp3d estimate
  double min_energy = std::numeric_limits<double>::max();
  Pose pwp3d_best_pose = current_model.CurrentPose();
  
  //iterate until convergence
  for(int step=0; step < NUM_STEPS && !converged; step++){

    
    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    double energy = 0.0;
    
    for(int eye=0; ;eye++){

      if(!SetupEye(eye,current_model.CurrentPose())) break; //sets the camera_ variable to left or right eye breaks after resetting everything back to initial state after right eye
  
      cv::Mat sdf_image = ProjectShapeToSDF(current_model);

      //compute the normalization values n_f and n_b
      double norm_foreground,norm_background;
      ComputeNormalization(norm_foreground,norm_background,sdf_image);
      if(norm_foreground == 0) {
#ifdef DEBUG
        std::cerr << "The object is not in view!\n"; 
#endif
        return current_model.CurrentPose();
      }



      //compute the derivates of the sdf images
      cv::Mat dSDFdx, dSDFdy;
      cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
      cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

      for(int r=0;r<frame_->GetImageROI().rows;r++){
        for(int c=0;c<frame_->GetImageROI().cols;c++){

          //speedup tests by checking if we need to evaluate the cost function in this region
          const double skip = Heaviside(sdf_image.at<float>(r,c));
          if( skip < 0.0001 || skip > 0.99999 ) continue;

          //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/progress
          energy += GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background);

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c), norm_foreground, norm_background);
            
          //dH / dL
          const cv::Mat pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);

          //update the jacobian
          for(int i=0;i<pose_derivatives.rows;i++){
            double pp = (region_agreement*pose_derivatives.at<double>(i,0));
            if (pp != pp) continue;
            jacobian.at<double>(i,0) += -1 * (region_agreement*pose_derivatives.at<double>(i,0));
          }

        }
      }
    

#ifdef SAVEDEBUG
    ENERGY_FILE << energy << " ";
    ENERGY_FILE.flush();
    std::cout << "ENERGY IS : " << energy << std::endl;
#endif
    
    //if(energy < min_energy) {
     min_energy = energy;
     //pwp3d_best_pose = current_model.CurrentPose();
     
    //}
    }   
    //update the pose estimate
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose(),step);
    
     /*
    //do point based registration
    std::vector<MatchedPair> pnp_pairs;
    //FindPointCorrespondences(frame_,pnp_pairs);
    FindPointCorrespondencesWithPose(frame_,pnp_pairs,current_model.CurrentPose());
    std::cerr << "Matching to " << pnp_pairs.size() << " points\n";
    for(auto pnp=pnp_pairs.begin();pnp!=pnp_pairs.end();pnp++){
      cv::Mat pnp_jacobian = GetPointDerivative(cv::Point3f(current_model.CurrentPose().Transform(pnp->learned_point)),cv::Point2f(pnp->image_point[0],pnp->image_point[1]), current_model.CurrentPose());
      for(int i=0;i<jacobian.rows;i++){
        //continue;
        jacobian.at<double>(i,0) += 0.05 * pnp_jacobian.at<double>(i,0);
      }
    }
    */

    //save the step estimate
#ifdef SAVEDEBUG
    cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
    cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
    DrawModelOnFrame(current_model,left_canvas,right_canvas);
    std::stringstream ss_2; ss_2 << "step_" << step << ".png";
    cv::imwrite(ss.str()+"/left/"+ss_2.str(),left_canvas);
    cv::imwrite(ss.str()+"/right/"+ss_2.str(),right_canvas);
#endif

  //test for convergence
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose() );
    converged = HasGradientDescentConverged__new(convergence_test_values, jacobian );



  }

#ifdef SAVEDEBUG
  ENERGY_FILE.close();
#endif
  
  
  /********************************************************************/
  /*
  Pose point_registration_pose = ApplyPointBasedRegistration(frame,current_model);
  
  //test if the point based regisration was better than the region based alignment
  double point_energy = 0.0;
  {
    current_model.CurrentPose() = point_registration_pose;
    cv::Mat sdf_image = ProjectShapeToSDF(current_model);

    //compute the normalization values n_f and n_b
    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);
    if(norm_foreground == 0) {
      std::cerr << "Error, the point tracking gave a terrible result!\n";
      return pwp3d_best_pose;
    }

    //compute the derivates of the sdf images
    cv::Mat dSDFdx, dSDFdy;
    cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1);
    cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);

    //cv::Mat ENERGY_IMAGE = cv::Mat::zeros(ROI_left_.size(),CV_32FC1);
   

    for(int r=0;r<ROI_left_.rows;r++){
      for(int c=0;c<ROI_left_.cols;c++){

        if(!stereo_frame->InsideRectifiedRegion(r,c)) continue;
        double skip = Heaviside(sdf_image.at<float>(r,c));
        if( skip < 0.0001 || skip > 0.99999 ){
          //skip_im.at<unsigned char>(r,c) = 255;
          continue;
        }
        //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/progress
        point_energy += GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background);

      }
    }

#ifdef SAVEDEBUG
    cv::Mat canvas = frame_->Mat().clone();
    DrawModelOnFrame(current_model,canvas);
    cv::imwrite(ss.str()+"/point_registration.png",canvas);
#endif


  }
  */
  //if (min_energy < point_energy){

  //current_model.CurrentPose() = pwp3d_best_pose;
  std::cerr << "Current pose = " << cv::Point3f(current_model.CurrentPose().translation_) << " -- " << current_model.CurrentPose().rotation_ << "\n";
  std::cerr << "MIN_ENERGY = " << min_energy << "\n";
  //std::cerr << "POINT ENERGY = " << point_energy << "\n";
  std::cerr << "Returning pwp3d energy!\n";

  //}else{

  //  current_model.CurrentPose() = point_registration_pose;

  //  std::cerr << "MIN_ENERGY = " << min_energy << "\n";
  //  std::cerr << "POINT ENERGY = " << point_energy << "\n";
   // std::cerr << "Retuing point energy!\n";
    
  //}

  /**********************************************************************/
  
  //update the velocity model... a bit crude
  cv::Vec3f translational_velocity = current_model.CurrentPose().translation_ - initial_translation;
  current_model.CurrentPose().translational_velocity_ = translational_velocity;
  return current_model.CurrentPose();

}

void StereoPWP3D::FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &pnp, const Pose &pose){

  //load the ground truth points from the file
  std::vector<Descriptor> ground_truth_descriptors;
  ReadKeypoints("Keypoints.xml",ground_truth_descriptors,NUM_DESCRIPTOR); 
  //transform them to the current coordinate system
  for(auto kp = ground_truth_descriptors.begin(); kp != ground_truth_descriptors.end(); kp++){

    kp->coordinate = pose.Transform(kp->coordinate);

  }
  
  //search the image plane for features to match
  std::vector<Descriptor> frame_descriptors;
  GetDescriptors(frame->GetImageROI(),frame_descriptors);
  
  //for each keypoint
  for(auto kp=ground_truth_descriptors.begin(); kp != ground_truth_descriptors.end(); kp++){

    std::vector<std::pair<Descriptor, double> > matching_queue;
    //project them to the image plane
    cv::Point2f projected_pt = stereo_camera_->rectified_left_eye()->ProjectPointToPixel(cv::Point3f(kp->coordinate));
    
    //iterate over the found features
    for(auto frame_descriptor = frame_descriptors.begin(); frame_descriptor != frame_descriptors.end(); frame_descriptor++){

      cv::Point2f pt_to_match(frame_descriptor->coordinate[0],frame_descriptor->coordinate[1]);
      
      //if the euclidean distance is < threshold then add this point to matching vector
      double euclidean_distance = std::sqrt((projected_pt.x - pt_to_match.x)*(projected_pt.x - pt_to_match.x) + (projected_pt.y - pt_to_match.y)*(projected_pt.y - pt_to_match.y));
      if(euclidean_distance < MATCHING_DISTANCE_THRESHOLD) matching_queue.push_back(std::pair<Descriptor,double>(*frame_descriptor,0.0));

    }
    
    if( !matching_queue.size() ) continue; //no matches found :(
    
    for(auto mq=matching_queue.begin(); mq != matching_queue.end(); mq++){

      mq->second = l2_norm(kp->descriptor,mq->first.descriptor);

    }

    std::sort(matching_queue.begin(),matching_queue.end(),[](const std::pair<Descriptor,double>& before, const std::pair<Descriptor,double>& after) -> bool
    {
      return before.second < after.second;
    });
    //run l2 norm based matching between learned point and points in this vector. is the matching score is good enough, add to the matches
    
    double size_of_best = matching_queue.front().second;//l2_norm( matching_queue.front().first.descriptor, cv::Mat::zeros(matching_queue.front().first.descriptor.size(),matching_queue.front().first.descriptor.type()));  
    
    if(size_of_best < DESCRIPTOR_SIMILARITY_THRESHOLD){
      cv::Point2f pt_to_match(matching_queue.front().first.coordinate[0],matching_queue.front().first.coordinate[1]);
      if(std::abs(pt_to_match.x - projected_pt.x) < 2 && std::abs(pt_to_match.y - projected_pt.y) < 2)
        cv::line(frame->GetImageROI(),pt_to_match,projected_pt,cv::Scalar(244,0,10),3);
      else
        cv::circle(frame->GetImageROI(),pt_to_match,4,cv::Scalar(244,0,10),2);
      MatchedPair mp;
      mp.learned_point = kp->coordinate;
      mp.image_point = matching_queue.front().first.coordinate;
      pnp.push_back( mp );
    }

  }
  
  //cv::imwrite("test_point.png",frame->Mat());
  
  
  //return matched vector




}

cv::Mat StereoPWP3D::GetPointDerivative(const cv::Point3f &world, cv::Point2f &image, const Pose &pose) const{
  
  const int NUM_DERIVS = 7;
  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  cv::Vec3f front_intersection(world);
   
  if(front_intersection[2] == 0.0) front_intersection[2] = 0.001;
  double z_inv_sq = 1.0/front_intersection[2];
  
  cv::Point2f projected_world = stereo_camera_->rectified_left_eye()->ProjectPointToPixel(world);

  for(int dof=0;dof<NUM_DERIVS;dof++){

    const cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,pose,cv::Vec3f(world));
      
    const double dXdL = stereo_camera_->rectified_left_eye()->Fx() * (z_inv_sq*((front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2])));
    const double dYdL = stereo_camera_->rectified_left_eye()->Fy() * (z_inv_sq*((front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2])));
    ret.at<double>(dof,0) = 2*((image.x - projected_world.x)*dXdL + (image.y - projected_world.y)*dYdL);
      
  }

  return ret;



}


bool StereoPWP3D::HasGradientDescentConverged__new(std::vector<cv::Mat> &convergence_test_values, cv::Mat &current_estimate) const {

  convergence_test_values.push_back(current_estimate);

  if (current_estimate.type() != CV_64FC1 ) throw(std::runtime_error("Set Jacobian values to double!\n"));
  if (current_estimate.cols != 1) throw(std::runtime_error("Error, jacobian should be Nx1 vector!\n"));

  if(convergence_test_values.size() < 6) 
    return false;

  std::vector<cv::Mat> last_5_values(convergence_test_values.end()-5,convergence_test_values.end());
  cv::Mat sum_vals = cv::Mat::zeros(current_estimate.size(),current_estimate.type());
  
  for(auto jac = last_5_values.begin() ; jac!=last_5_values.end() ; jac++){

    sum_vals = sum_vals + *jac;

  }

  double jac_l2_norm = 0.0;
  for(int r=0;r<sum_vals.rows;r++){
    jac_l2_norm += (sum_vals.at<double>(r)*sum_vals.at<double>(r));
  }

  jac_l2_norm = sqrt(jac_l2_norm);

  std::cerr << "Jacobian L2 Norm = " << jac_l2_norm << "\n";

  if(jac_l2_norm < 3.8e6){
    std::cerr << "Convergence reached in " << convergence_test_values.size() << "!\n";
    return true;
  }

  return false;

}

bool StereoPWP3D::HasGradientDescentConverged(std::vector<Pose> &convergence_test_values, Pose &current_estimate) const {

  convergence_test_values.push_back(current_estimate);

  //if (current_estimate.type() != CV_64FC1 ) throw(std::runtime_error("Set Jacobian values to double!\n"));
  //if (current_estimate.cols != 1) throw(std::runtime_error("Error, jacobian should be Nx1 vector!\n"));

  if(convergence_test_values.size() < 6) 
    return false;

  std::vector<double> rotation_change_between_iterations;
  std::vector<double>  translation_change_between_iterations;
  for(std::vector<Pose>::iterator pose_1 = convergence_test_values.begin(),pose_2 = convergence_test_values.begin()+1;
    pose_2 != convergence_test_values.end(); pose_1++,pose_2++) {
       
      const double rotational_distance = pose_1->rotation_.AngularDistanceToQuaternion(pose_2->rotation_);
      const cv::Vec3f translation_distance = (pose_1->translation_ - pose_2->translation_);
      const double translation_magnitude = sqrt(translation_distance[0]*translation_distance[0] + translation_distance[1]*translation_distance[1] + translation_distance[2]*translation_distance[2]);
  
      rotation_change_between_iterations.push_back(rotational_distance);
      translation_change_between_iterations.push_back(translation_magnitude);

  }

  rotation_change_between_iterations = std::vector<double>(rotation_change_between_iterations.end()-5,rotation_change_between_iterations.end());
  translation_change_between_iterations = std::vector<double>(translation_change_between_iterations.end()-5,translation_change_between_iterations.end());

  if(rotation_change_between_iterations.size() != 5 && rotation_change_between_iterations.size() != translation_change_between_iterations.size()){
    throw(std::runtime_error("Error, here!\n"));
  }

  //if(rotation_change_between_iterations.size() < 6) throw(std::runtime_error("Error, too few values in rotational change vector!\n"));

  double rot_sum = std::accumulate(rotation_change_between_iterations.begin(),
                                   rotation_change_between_iterations.end(),
                                   0.0);
  double rot_mean = rot_sum/rotation_change_between_iterations.size();
  //subtract the mean from the past 5 values and then sum them. if the values is sufficiently small, conclude convergence
  
  double rot_sq_sum = std::inner_product(rotation_change_between_iterations.begin(), rotation_change_between_iterations.end(), rotation_change_between_iterations.begin(), 0.0);
  double rot_stdev = std::sqrt(rot_sq_sum / rotation_change_between_iterations.size() - rot_mean * rot_mean);  

  double trans_sum = std::accumulate(translation_change_between_iterations.begin(),
                                   translation_change_between_iterations.end(),
                                   0.0);
  double trans_mean = trans_sum/translation_change_between_iterations.size();  
  double trans_sq_sum = std::inner_product(translation_change_between_iterations.begin(), translation_change_between_iterations.end(), translation_change_between_iterations.begin(), 0.0);
  double trans_stdev = std::sqrt(trans_sq_sum / translation_change_between_iterations.size() - trans_mean * trans_mean);
  
  //std::cerr << "A set of rotational values:\n";
  //for(auto r=rotation_change_between_iterations.begin();r!=rotation_change_between_iterations.end();r++){
    //std::cerr << *r << " --> ";
    //*r = *r - rot_mean;
    //std::cerr << *r << "\n";
  //}

  //\rot_sum = std::accumulate(rotation_change_between_iterations.begin(),rotation_change_between_iterations.end(),0.0);

  std::cerr << "Rotation sum is " << rot_sum << std::endl;
  //std::cerr << "A set of translational values:\n";
  for(auto t=translation_change_between_iterations.begin();t!=translation_change_between_iterations.end();t++){
    //std::cerr << *t << " --> ";
    *t = *t - trans_mean;
    //std::cerr << *t << "\n";
  }

  
  trans_sum = std::accumulate(translation_change_between_iterations.begin(),translation_change_between_iterations.end(),0.0);

  
  //std::cerr << "Translation sum is " << trans_sum << std::endl;

  /*if(rot_stdev < 0.0013 && trans_stdev < 0.013){
    std::cerr << "Convergence reached!\n" << rot_stdev << " and " << trans_stdev << "\n";
  }*/

  if(std::abs(rot_sum) < 1e-19 && std::abs(trans_sum) < 0.012){
    std::cerr << "\n\n";
    std::cerr << "Convergence reached!\n" << rot_stdev << " and " << trans_stdev << "\n";
    std::cerr << "\n\n";
  }
  return false;
}

void ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count){

  cv::FileStorage ifs(filename, cv::FileStorage::READ);
  if(!ifs.isOpened()) {
    throw(std::runtime_error("ERror could not open file!\n"));
  }
  
  for(int n=0; n<count;n++){

   
    Descriptor ds;
    ds.read(ifs,n);
    descriptors.push_back(ds);
    
  }
    
  if (descriptors.size() < 4) {
    throw(std::runtime_error("Error, could not find more than 3 good descriptors in file!\n"));
  }
}



void StereoPWP3D::FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair){

  std::vector<Descriptor> ds;
  ReadKeypoints("Keypoints.xml",ds,NUM_DESCRIPTOR); 
  cv::Mat descriptors;
  for(auto d = ds.begin(); d != ds.end() ; d++){
    descriptors.push_back(d->descriptor);
  }
  
  std::vector<Descriptor> left_ds;
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
  GetDescriptors( stereo_frame->GetLeftImage() , left_ds);
  std::vector<DescriptorMatches> matches;
  MatchDescriptorsToModel(ds,left_ds,matches);
  for(auto dm = matches.begin();dm!=matches.end();dm++){

    MatchedPair mp;
    mp.image_point = dm->left_image.coordinate;
    mp.learned_point = dm->gt.coordinate;
    matched_pair.push_back( mp );

  }


}



Pose StereoPWP3D::ApplyPointBasedRegistration(boost::shared_ptr<sv::Frame> frame, KalmanTracker &current_model ){


  std::vector<Descriptor> ds;
  ReadKeypoints("Keypoints.xml",ds,NUM_DESCRIPTOR); 
  cv::Mat descriptors;
  for(auto d = ds.begin(); d != ds.end() ; d++){
    descriptors.push_back(d->descriptor);
  }

  std::vector<Descriptor> left_ds;
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
  GetDescriptors( stereo_frame->GetLeftImage() , left_ds);
  std::vector<DescriptorMatches> matches;
  MatchDescriptorsToModel(ds,left_ds,matches);

  //std::vector<Descriptor> right_ds;
  //GetDescriptors(stereo_frame->RightMat(), right_ds);

  //FindCorrespondingMatches(right_ds,matches);

  //std::vector<MatchedPair> matched_pair;
  //TriangulateMatches(matches,matched_pair,stereo_camera_,stereo_frame->LeftMat(),stereo_frame->RightMat());

  cv::Mat rotation,translation;
  //FindTransformation(matched_pair,rotation,translation);
  FindTransformationToImagePlane(matches,rotation,translation,stereo_camera_,current_model.CurrentPose());
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation,rotation_matrix);
  Pose pose(translation,sv::Quaternion(rotation_matrix));
  //current_model.CurrentPose() = pose;
  return pose;
}

void FindTransformationToImagePlane(std::vector<DescriptorMatches> matches,cv::Mat &rotation, cv::Mat &translation,  boost::shared_ptr<StereoCamera> cam, Pose current_pose){

  std::vector<cv::Point3f> world_points;
  std::vector<cv::Point2f> image_points;

  for(auto pair = matches.begin();pair!=matches.end();pair++){

    world_points.push_back(cv::Point3f(pair->gt.coordinate));
    image_points.push_back(cv::Point2f(pair->left_image.coordinate[0],pair->left_image.coordinate[1]));

  }
  cv::Rodrigues(current_pose.rotation_.AngleAxis(),rotation);
  translation = cv::Mat(1,3,CV_64FC1);
  for(int i=0;i<3;i++) translation.at<double>(i) = current_pose.translation_[i];
  cv::solvePnPRansac(world_points,image_points,cam->rectified_left_eye()->intrinsic_params(),cam->rectified_left_eye()->distortion_params(),rotation,translation,true);
}

void FindTransformation(std::vector<MatchedPair> &matched_pair, cv::Mat &rotation, cv::Mat &translation){

  //cv::Mat src_points,dst_points;
  std::vector<cv::Point3f> src_points,dst_points;
  for (auto pair = matched_pair.begin(); pair != matched_pair.end(); pair++ ){

    //cv::Mat pt1(1,3,CV_32FC1),pt2(1,3,CV_32FC1);
    cv::Point3f pt1(pair->learned_point),pt2(pair->image_point);
    //for(int i=0;i<3;i++){
    //  pt1.at<float>(i) = pair->learned_point[i];
    //  pt2.at<float>(i) = pair->image_point[i];
    //}

    src_points.push_back(pt1);
    dst_points.push_back(pt2);

  }

  std::vector<unsigned char> inliers;
  cv::Mat affine(3,4,CV_32FC1);//,inl iers(src_points.size(),src_points.type());
  cv::estimateAffine3D(src_points,dst_points,affine,inliers);
  std::cerr << affine << "\n";

  //}

}

void GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds){
  cv::Mat image_gray;
  cv::cvtColor(frame,image_gray,CV_RGB2GRAY);
  cv::SiftFeatureDetector detector;//(400);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image_gray,keypoints);
  cv::SiftDescriptorExtractor extractor;
  cv::Mat descriptors_image;
  extractor.compute(image_gray,keypoints,descriptors_image);
  int i = 0;
  for (auto kp = keypoints.begin(); kp != keypoints.end() ; kp++){
    Descriptor d;
    d.coordinate = cv::Vec3f(kp->pt.x,kp->pt.y,0);
    d.descriptor = descriptors_image.row(i);
    i++;
    ds.push_back(d);
  }
}

void MatchDescriptorsToModel(std::vector<Descriptor> &model_descriptors, std::vector<Descriptor> &image_descriptors, std::vector<DescriptorMatches> &found_matches){
  
  cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector<cv::DMatch> matches;
  cv::Mat model_descriptor_matrix,image_descriptor_matrix;

  for(auto d = model_descriptors.begin(); d != model_descriptors.end() ; d++ )
    model_descriptor_matrix.push_back(d->descriptor);

  for(auto d = image_descriptors.begin(); d != image_descriptors.end() ; d++ )
    image_descriptor_matrix.push_back(d->descriptor);
  
  /* match( queryMatrix, traingMatrix ) */
  matcher.match(model_descriptor_matrix, image_descriptor_matrix, matches);
  
  for (auto match = matches.begin(); match != matches.end() ; match++ ){
    
    DescriptorMatches d;

    d.gt = model_descriptors[match->queryIdx];
    d.left_image = image_descriptors[match->trainIdx];
    d.right_image.coordinate = cv::Vec3f(0,0,0);
    found_matches.push_back(d);

    //cv::Point2i original(cam->rectified_left_eye()->ProjectPointToPixel(pose.Transform(d.gt.coordinate)));
    //cv::Point2i found(d.left_image.coordinate[0]+5,d.left_image.coordinate[1]);
    //cv::circle(left, original, 6, cv::Scalar(0,255,0), 3);
    //cv::circle(left, found, 6, cv::Scalar(0,25,63),3 );
    //cv::line(left, original,found , cv::Scalar(255,0,10), 10);    

  }

}

void FindCorrespondingMatches(std::vector<Descriptor> &right_image_descriptors, std::vector<DescriptorMatches> &left_image_matches){
  
  cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector<cv::DMatch> matches;
  cv::Mat right_image_descriptor_matrix,left_image_descriptor_matrix;
  for(auto d = right_image_descriptors.begin(); d != right_image_descriptors.end() ; d++ )
    right_image_descriptor_matrix.push_back(d->descriptor);
  for(auto d = left_image_matches.begin(); d != left_image_matches.end() ; d++ )
    left_image_descriptor_matrix.push_back(d->left_image.descriptor);
    
  /* match( queryMatrix, traingMatrix ) */
  matcher.match(right_image_descriptor_matrix, left_image_descriptor_matrix, matches);

  std::sort(matches.begin(),matches.end(),
      [](const cv::DMatch &a, const cv::DMatch &b) -> bool {
        return a.distance < b.distance;
    });

  matches.erase(matches.begin()+20,matches.end());

  for (auto match = matches.begin(); match != matches.end() ; match++ ){

    DescriptorMatches &dm = left_image_matches[match->trainIdx];
    Descriptor &d = right_image_descriptors[match->queryIdx];
    dm.right_image = d;
  
  }

}

void TriangulateMatches(std::vector<DescriptorMatches> &matches, std::vector<MatchedPair> &matched_pair, boost::shared_ptr<StereoCamera> cam, cv::Mat &left, cv::Mat &right){

  cv::Mat both(left.rows+right.rows,left.cols,CV_8UC3);
  cv::Mat lsec = both.colRange(0,left.cols).rowRange(0,left.rows);
  left.copyTo(lsec);
  //cv::Mat rsec = both.colRange(left.cols,left.cols*2).rowRange(0,left.rows);//(cv::Rect(left.cols,0,left.cols,left.rows));
  cv::Mat rsec = both.colRange(0,left.cols).rowRange(left.rows,2*left.rows);//(cv::Rect(left.cols,0,left.cols,left.rows));
  right.copyTo(rsec);
  
  for(auto match = matches.begin(); match != matches.end() ; match++ ){

    if(match->right_image.coordinate == cv::Vec3f(0,0,0)) 
      continue;

    cv::Vec3f &l = match->left_image.coordinate;
    cv::Vec3f &r = match->right_image.coordinate;

    cv::Point2i l_pt(l[0],l[1]);
    cv::Point2i r_pt(r[0],r[1]);
    cv::Point2i stereo(r_pt.x,r_pt.y+left.rows);


    cv::Vec3f triangulated_point = cam->ReprojectPointTo3D(l_pt,r_pt);  
    //std::cerr << cv::Point3f(triangulated_point) << "\n";// << " -- > " << cv::Point3f(match->gt.coordinate) << "\n";
    if(triangulated_point == cv::Vec3f(0,0,0)) continue;
    //if(triangulated_point == cv::Vec3f(0,0,0) ) continue;
    //cv::circle(left, l_pt, 5, cv::Scalar(0,20,245));
    //cv::circle(right, r_pt, 5, cv::Scalar(2,20,245));
    cv::line(both,l_pt,stereo,cv::Scalar(24,245,24),5);


    MatchedPair mp;
    mp.image_point = triangulated_point;
    mp.learned_point = match->gt.coordinate;
    matched_pair.push_back(mp);

  }

  
  //cv::imwrite("LeftMatch.png",left);
  //cv::imwrite("RIghtMatch.png",right);
  cv::imwrite("BOTH.png",both);

}

void StereoPWP3D::ComputeDescriptorsForPointTracking(boost::shared_ptr<sv::Frame> frame, KalmanTracker current_model ){

    //make a descriptor finder
    cv::Mat image_gray;
    cv::cvtColor(frame_->GetImageROI(),image_gray,CV_RGB2GRAY);
    cv::Mat shape_image = ProjectShapeToSDF(current_model);

    cv::SiftFeatureDetector detector;//(400);
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(image_gray,keypoints);
    std::vector<cv::KeyPoint> tool_keypoints;

    for( auto kp = keypoints.begin(); kp != keypoints.end(); kp++ ){

      cv::Point2f &pt = kp->pt;
      if(shape_image.at<float>(pt.y,pt.x) > 0){
        tool_keypoints.push_back(*kp);
      }
    
    }

    std::sort(tool_keypoints.begin(),tool_keypoints.end(),
      [](const cv::KeyPoint &a, const cv::KeyPoint &b) -> bool {
        return a.response > b.response;
    });

    cv::FileStorage fs("KeyPoints.xml",cv::FileStorage::WRITE);
    //collect descriptors in the image plane
    //cv::SurfDescriptorExtractor extractor;
    cv::SiftDescriptorExtractor extractor;
    cv::Mat descriptors;
    extractor.compute(image_gray,tool_keypoints,descriptors);
    int i=0;
    
    for( auto kp = tool_keypoints.begin(); kp != tool_keypoints.end() && kp != tool_keypoints.begin()+NUM_DESCRIPTOR; kp++ ){

      cv::Point2f &pt = kp->pt;
      cv::Vec3f ray = camera_->UnProjectPoint( cv::Point2i(int(pt.x),int(pt.y)) );
      cv::Vec3f front;
      current_model.PtrToModel()->GetIntersection(ray, front, cv::Vec3f() ,current_model.CurrentPose());
      
      cv::Vec3f front_on_model = current_model.CurrentPose().InverseTransform(front);
        
      Descriptor ds;
      ds.coordinate = front_on_model;
      ds.descriptor = descriptors.row(i);
      ds.write(fs,i);     
      i++;
    
    }
}
