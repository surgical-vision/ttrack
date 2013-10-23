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

    Camera() = GetStereoCamera()->rectified_right_eye();

    //swap the roi over in the images so they refer to the right hand image
    stereo_frame->SwapEyes();
    //also update the object pose so that it's relative to the right eye
    Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    pose = CombinePoses(extrinsic, pose);

    return true;

  }else{

    Camera() = GetStereoCamera()->rectified_left_eye();

    //swap everything back
    stereo_frame->SwapEyes();

    Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
    pose = CombinePoses(extrinsic.Inverse(),pose);

    return false;

  }

}

cv::Mat StereoPWP3D::GetPoseDerivativesRightEye(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model){

  const int NUM_DERIVS = 7;
     
   //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;
  bool intersects = GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  
  //because the derivative only works for points which project to the target and we need it to be defined for points outside the contour, 'pretend' that a small region of these points actually hit the contour
  if(!intersects) {
    intersects = GetNearestIntersection(r,c,sdf,front_intersection,back_intersection,current_model);
    if(!intersects)
      throw(std::runtime_error("Error, should not miss point on the border. Check GetNearestIntesection::search_range and continue values!\n"));
      //return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
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
    const cv::Vec3f dof_derivatives_back = GetDOFDerivatives(dof,current_model.CurrentPose(),back_intersection);

    const double dXdL = camera_->Fx() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[0]) - (front_intersection[0]*dof_derivatives_front[2]))) + camera_->Fx() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[0]) - (back_intersection[0]*dof_derivatives_back[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq_front*((front_intersection[2]*dof_derivatives_front[1]) - (front_intersection[1]*dof_derivatives_front[2]))) + camera_->Fy() * (z_inv_sq_back*((back_intersection[2]*dof_derivatives_back[1]) - (back_intersection[1]*dof_derivatives_back[2])));
    ret.at<double>(dof,0) = DeltaFunction(sdf.at<float>(r,c)) * ((dSDFdx * dXdL) + (dSDFdy * dYdL));
      
  }
  return ret;

}


cv::Vec3f StereoPWP3D::GetDOFDerivativesRightEye(const int dof, const Pose &pose, const cv::Vec3f &point_) {

  //derivatives use the (x,y,z) from the initial reference frame not the transformed one so inverse the transformation
  //cv::Vec3f point = point_ - pose.translation_;
  //point = pose.rotation_.Inverse().RotateVector(point);
  Pose extrinsic(cv::Vec3f(stereo_camera_->ExtrinsicTranslation()[0],0,0),sv::Quaternion(cv::Mat::eye(3,3,CV_64FC1)));
  cv::Vec3f point = CombinePoses(extrinsic.Inverse(),pose).Transform(point_);

  cv::Mat extrinsic_rotation = cv::Mat::eye(3,3,CV_64FC1);

  switch(dof){

  case 0: //x
    return cv::Vec3f(
      extrinsic_rotation.at<double>(0,0),
      0,
      0
      );
  case 1: //y
    return cv::Vec3f(
      0,
      extrinsic_rotation.at<double>(1,1),
      0
      );
  case 2: //z
    return cv::Vec3f(
      0,
      0,
      extrinsic_rotation.at<double>(2,2)
      );


  case 3: //qw
    return cv::Vec3f(
      extrinsic_rotation.at<double>(0,0)*((2*pose.rotation_.Y()*point[2])-(2*pose.rotation_.Z()*point[1])) + 
      extrinsic_rotation.at<double>(0,1)*((2*pose.rotation_.Z()*point[0])-(2*pose.rotation_.X()*point[2])) + 
      extrinsic_rotation.at<double>(0,2)*((2*pose.rotation_.X()*point[1])-(2*pose.rotation_.Y()*point[0])),

      extrinsic_rotation.at<double>(1,0)*((2*pose.rotation_.Y()*point[2])-(2*pose.rotation_.Z()*point[1])) + 
      extrinsic_rotation.at<double>(1,1)*((2*pose.rotation_.Z()*point[0])-(2*pose.rotation_.X()*point[2])) + 
      extrinsic_rotation.at<double>(1,2)*((2*pose.rotation_.X()*point[1])-(2*pose.rotation_.Y()*point[0])),
      
      extrinsic_rotation.at<double>(2,0)*((2*pose.rotation_.Y()*point[2])-(2*pose.rotation_.Z()*point[1])) + 
      extrinsic_rotation.at<double>(2,1)*((2*pose.rotation_.Z()*point[0])-(2*pose.rotation_.X()*point[2])) + 
      extrinsic_rotation.at<double>(2,2)*((2*pose.rotation_.X()*point[1])-(2*pose.rotation_.Y()*point[0]))
      );

  case 4: //qx
    return cv::Vec3f(
      
      extrinsic_rotation.at<double>(0,0)*((2*pose.rotation_.Y()*point[1])+(2*pose.rotation_.Z()*point[2])) +
      extrinsic_rotation.at<double>(0,1)*((2*pose.rotation_.Y()*point[0])-(4*pose.rotation_.X()*point[1])-(2*pose.rotation_.W()*point[2])) + 
      extrinsic_rotation.at<double>(0,2)*((2*pose.rotation_.Z()*point[0])+(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.X()*point[2])),

      extrinsic_rotation.at<double>(1,0)*((2*pose.rotation_.Y()*point[1])+(2*pose.rotation_.Z()*point[2])) +
      extrinsic_rotation.at<double>(1,1)*((2*pose.rotation_.Y()*point[0])-(4*pose.rotation_.X()*point[1])-(2*pose.rotation_.W()*point[2])) + 
      extrinsic_rotation.at<double>(1,2)*((2*pose.rotation_.Z()*point[0])+(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.X()*point[2])),

      extrinsic_rotation.at<double>(2,0)*((2*pose.rotation_.Y()*point[1])+(2*pose.rotation_.Z()*point[2])) +
      extrinsic_rotation.at<double>(2,1)*((2*pose.rotation_.Y()*point[0])-(4*pose.rotation_.X()*point[1])-(2*pose.rotation_.W()*point[2])) + 
      extrinsic_rotation.at<double>(2,2)*((2*pose.rotation_.Z()*point[0])+(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.X()*point[2]))
      
      );

  case 5: //qy
    return cv::Vec3f(
       extrinsic_rotation.at<double>(0,0)*((2*pose.rotation_.X()*point[1])-(4*pose.rotation_.Y()*point[0])+(2*pose.rotation_.W()*point[2])) +
       extrinsic_rotation.at<double>(0,1)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Z()*point[2])) +
       extrinsic_rotation.at<double>(0,2)*((2*pose.rotation_.Z()*point[1])+(2*pose.rotation_.W()*point[0])-(4*pose.rotation_.Y()*point[2])),
      
       extrinsic_rotation.at<double>(1,0)*((2*pose.rotation_.X()*point[1])-(4*pose.rotation_.Y()*point[0])+(2*pose.rotation_.W()*point[2])) +
       extrinsic_rotation.at<double>(1,1)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Z()*point[2])) +
       extrinsic_rotation.at<double>(1,2)*((2*pose.rotation_.Z()*point[1])+(2*pose.rotation_.W()*point[0])-(4*pose.rotation_.Y()*point[2])),
       
       extrinsic_rotation.at<double>(2,0)*((2*pose.rotation_.X()*point[1])-(4*pose.rotation_.Y()*point[0])+(2*pose.rotation_.W()*point[2])) +
       extrinsic_rotation.at<double>(2,1)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Z()*point[2])) +
       extrinsic_rotation.at<double>(2,2)*((2*pose.rotation_.Z()*point[1])+(2*pose.rotation_.W()*point[0])-(4*pose.rotation_.Y()*point[2]))
      );

  case 6: //qz
    return cv::Vec3f(
      extrinsic_rotation.at<double>(0,0)*((2*pose.rotation_.X()*point[2])-(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.Z()*point[0])) + 
      extrinsic_rotation.at<double>(0,1)*((2*pose.rotation_.W()*point[0])-(4*pose.rotation_.X()*point[1])+(2*pose.rotation_.Y()*point[2])) +
      extrinsic_rotation.at<double>(0,2)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Y()*point[1])),

      extrinsic_rotation.at<double>(1,0)*((2*pose.rotation_.X()*point[2])-(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.Z()*point[0])) + 
      extrinsic_rotation.at<double>(1,1)*((2*pose.rotation_.W()*point[0])-(4*pose.rotation_.X()*point[1])+(2*pose.rotation_.Y()*point[2])) +
      extrinsic_rotation.at<double>(1,2)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Y()*point[1])),

      extrinsic_rotation.at<double>(2,0)*((2*pose.rotation_.X()*point[2])-(2*pose.rotation_.W()*point[1])-(4*pose.rotation_.Z()*point[0])) + 
      extrinsic_rotation.at<double>(2,1)*((2*pose.rotation_.W()*point[0])-(4*pose.rotation_.X()*point[1])+(2*pose.rotation_.Y()*point[2])) +
      extrinsic_rotation.at<double>(2,2)*((2*pose.rotation_.X()*point[0])+(2*pose.rotation_.Y()*point[1]))
      
      );

  default:
    throw std::runtime_error("Error, a value in the range 0-6 must be supplied");
  }


}

void StereoPWP3D::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) {

  std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();

  for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

    cv::Vec2f projected = camera_->ProjectPoint(point->vertex_);

    for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){

      const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
      cv::Vec2f projected_neighbour = camera_->ProjectPoint( neighbour.vertex_ );

      if(canvas.channels() == 3)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),cv::Scalar(255,0,255),1,CV_AA);
      if(canvas.channels() == 1)
        line(canvas,cv::Point2f(projected),cv::Point2f(projected_neighbour),(unsigned char)255,1,CV_AA);
    }
  }
}




void StereoPWP3D::DrawModelOnBothFrames(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas) {

  {
    std::vector<SimplePoint<> > transformed_points = tracked_model.ModelPointsAtCurrentPose();

    for(auto point = transformed_points.begin(); point != transformed_points.end(); point++ ){

      cv::Vec2f projected =  GetStereoCamera()->rectified_left_eye()->ProjectPoint(point->vertex_);

      for(auto neighbour_index = point->neighbours_.begin(); neighbour_index != point->neighbours_.end(); neighbour_index++){

        const SimplePoint<> &neighbour = transformed_points[*neighbour_index];
        cv::Vec2f projected_neighbour = GetStereoCamera()->rectified_left_eye()->ProjectPoint( neighbour.vertex_ );

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
        cv::Vec2f projected_neighbour = GetStereoCamera()->rectified_right_eye()->ProjectPoint( neighbour.vertex_ );

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
  const int NUM_STEPS = 115;
  cv::Vec3f initial_translation = current_model.CurrentPose().translation_;

#ifdef DEBUG
  boost::progress_timer t; //timer prints time when it goes out of scope
#endif


#ifdef SAVEDEBUG
  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
  cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
  DrawModelOnBothFrames(current_model,left_canvas,right_canvas);
  static int frame_count = 0;
  boost::filesystem::create_directory("debug");
  std::stringstream ss; ss << "debug/frame_" << frame_count;
  boost::filesystem::create_directory(ss.str());
  boost::filesystem::create_directory(ss.str()+"/left");
  boost::filesystem::create_directory(ss.str()+"/right");
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

  ComputeDescriptorsForPointTracking(frame_,current_model);

  //store a vector of Pose values. check std. dev. of these values, if this is small enough, assume convergence.
  std::vector<double> convergence_test_values;
  bool converged = false;

  //values to hold the 'best' pwp3d estimate
  double min_energy = std::numeric_limits<double>::max();
  Pose pwp3d_best_pose = current_model.CurrentPose();

  //iterate until convergence
  for(int step=0; step < NUM_STEPS && !converged; step++){

#ifdef SAVEDEBUG

    std::stringstream step_dir; step_dir << "step" << step;
    boost::filesystem::create_directory(ss.str()+"/left/"+step_dir.str());
    boost::filesystem::create_directory(ss.str()+"/right/"+step_dir.str());

#endif

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    double energy = 0.0;
    double pixel_count = 0;

    for(int eye=0; ;eye++){

      if(!SetupEye(eye,current_model.CurrentPose())) break; //sets the camera_ variable to left or right eye breaks after resetting everything back to initial state after right eye
      
      cv::Mat sdf_image = ProjectShapeToSDF(current_model);

#ifdef SAVEDEBUG
      if(eye == 0){
        cv::imwrite(ss.str() + "/left/" + step_dir.str() + "/sdf.png",sdf_image);
        cv::Mat left_canvas = stereo_frame->GetLeftImage().clone();
        DrawModelOnFrame(current_model,left_canvas);
        cv::imwrite(ss.str()+"/left/"+step_dir.str()+"/previous.png",left_canvas);
      }
      if(eye == 1) {
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/sdf.png",sdf_image);
        cv::Mat right_canvas = stereo_frame->GetRightImage().clone();
        DrawModelOnFrame(current_model,right_canvas);
        cv::imwrite(ss.str()+"/right/"+step_dir.str()+"/previous.png",right_canvas);
      }
#endif

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
      //cv::Sobel(sdf_image,dSDFdx,CV_32FC1,1,0,1); // (src,dst,dtype,dx,dy,size) size = 1 ==> 3x1 finite difference kernel
      //cv::Sobel(sdf_image,dSDFdy,CV_32FC1,0,1,1);
      cv::Scharr(sdf_image,dSDFdx,CV_32FC1,1,0);
      cv::Scharr(sdf_image,dSDFdy,CV_32FC1,0,1);

#ifdef SAVEDEBUG
      cv::Mat jacobian_x = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat jacobian_y = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat jacobian_z = cv::Mat::zeros(sdf_image.size(),CV_8UC3);
      cv::Mat energy_image = cv::Mat::zeros(sdf_image.size(),CV_8UC1);
#endif

      std::cerr << "\n\nCurrently subsampling frame!\n\n";

      //for(int r=0;r<frame_->GetImageROI().rows;r++){
      //  for(int c=0;c<frame_->GetImageROI().cols;c++){
      for(int r=0;r<frame_->GetImageROI().rows;r+=3){
        for(int c=0;c<frame_->GetImageROI().cols;c+=3){
                  
          //if( c < 548 || c > 1390 ) continue;
          //if( r < 20  || r > 482  ) continue;

          //speedup tests by checking if we need to evaluate the cost function in this region
          const double skip = Heaviside(sdf_image.at<float>(r,c));
          if( skip < 0.00001 || skip > 0.99999 ) continue;

          //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/          
          energy += GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background); 
          energy_image.at<unsigned char>(r,c) = 255 * GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background);

          //P_f - P_b / (H * P_f + (1 - H) * P_b)
          const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c), norm_foreground, norm_background);

          //dH / dL
          cv::Mat pose_derivatives;
          if(eye == 0)
            pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);
          else if(eye == 1)
            pose_derivatives = GetPoseDerivativesRightEye(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);
          else
            throw(std::runtime_error("Error, bad stereo thing.\n"));

          //update the jacobian
          for(int i=0;i<pose_derivatives.rows;i++){
            double pp = (region_agreement*pose_derivatives.at<double>(i,0));
            if (pp != pp) continue;
            jacobian.at<double>(i,0) += -1 * (region_agreement*pose_derivatives.at<double>(i,0));
          }

          pixel_count++ ;
          
#ifdef SAVEDEBUG
          if(jacobian.at<double>(0,0) > 0)
            jacobian_x.at<cv::Vec3b>(r,c) = cv::Vec3b(255,0,0); //blue right
          else
            jacobian_x.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255); //red left

          if(jacobian.at<double>(1,0) > 0)
            jacobian_y.at<cv::Vec3b>(r,c) = cv::Vec3b(255,0,0); //blue down
          else
            jacobian_y.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255); //red up

          if(jacobian.at<double>(2,0) > 0)
            jacobian_z.at<cv::Vec3b>(r,c) = cv::Vec3b(255,0,0); //blue away
          else
            jacobian_z.at<cv::Vec3b>(r,c) = cv::Vec3b(0,0,255); //red towards
#endif
        }
      }
#ifdef SAVEDEBUG
      cv::Mat heaviside(jacobian_x.size(),CV_8UC1);
      cv::Mat delta(jacobian_x.size(),CV_8UC1);
      cv::Mat dSDFdx_save(jacobian_x.size(),CV_8UC3);
      cv::Mat dSDFdy_save(jacobian_x.size(),CV_8UC3);
      for(int r=0;r<heaviside.rows;r++){
        for(int c=0;c<heaviside.cols;c++){
          heaviside.at<unsigned char>(r,c) = 255 * Heaviside(sdf_image.at<float>(r,c));
          delta.at<unsigned char>(r,c) = 255 * DeltaFunction(sdf_image.at<float>(r,c));
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
      }else if(eye == 1) {
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/heaviside.png",heaviside);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/delta.png",delta);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_x.png",jacobian_x);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_y.png",jacobian_y);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/jacobian_z.png",jacobian_z);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/energy.png",energy_image);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/dsf_dx.png",dSDFdx_save);
        cv::imwrite(ss.str() + "/right/" + step_dir.str() + "/dsf_dy.png",dSDFdy_save);
      }
      


      ENERGY_FILE << energy << " ";
      ENERGY_FILE.flush();
      std::cout << "ENERGY IS : " << energy << std::endl;
#endif

      //if(energy < min_energy) {
      //  min_energy = energy;
      //  pwp3d_best_pose = current_model.CurrentPose();
      //}
    }   
    //update the pose estimate
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose(),step,pixel_count);

    convergence_test_values.push_back(energy);

    //do point based registration

    /*
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
    DrawModelOnBothFrames(current_model,left_canvas,right_canvas);
    cv::imwrite(ss.str()+"/left/"+step_dir.str()+"/update.png",left_canvas);
    cv::imwrite(ss.str()+"/right/"+step_dir.str()+"/update.png",right_canvas);
    cv::imwrite(ss.str()+"/left/"+step_dir.str()+".png",left_canvas);
    cv::imwrite(ss.str()+"/right/"+step_dir.str()+".png",right_canvas);
#endif

    //test for convergence
    //converged = HasGradientDescentConverged(convergence_test_values, current_model.CurrentPose() );
    //converged = HasGradientDescentConverged__new(convergence_test_values, jacobian );
    //converged = HasGradientDescentConverged(jacobian, current_model.CurrentPose());
    converged = HasGradientDescentConverged_UsingEnergy(convergence_test_values);
    std::cerr << "Current pose at end of gradient descent step " << step << " is:\n" << cv::Point3f(current_model.CurrentPose().translation_) << " -- " << current_model.CurrentPose().rotation_ << "\n";
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

bool StereoPWP3D::HasGradientDescentConverged(const cv::Mat &jacobian, const Pose &pose) const {

  double mag_pose = 0;
  for(int i=0;i<3;i++) mag_pose += pose.translation_[i]*pose.translation_[i];
  mag_pose += pose.rotation_.W()*pose.rotation_.W();
  mag_pose += pose.rotation_.X()*pose.rotation_.X();
  mag_pose += pose.rotation_.Y()*pose.rotation_.Y();
  mag_pose += pose.rotation_.Z()*pose.rotation_.Z();
  mag_pose = sqrt(mag_pose);

  double mag_jac = 0;
  for(int i=0;i<jacobian.rows;i++) mag_jac += jacobian.at<double>(i,0)*jacobian.at<double>(i,0);
  mag_jac = sqrt(mag_jac);

  return mag_pose > mag_jac * 15;
}

bool StereoPWP3D::HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const {

  const int NUM_VALUES_TO_USE = 7;
  if(energy_values.size() < NUM_VALUES_TO_USE ) return false;


  double energy_change = 0.0;
  for(auto value = energy_values.end()-(NUM_VALUES_TO_USE); value != energy_values.end()-1; value++ ){

    energy_change += *(value+1) - *(value);

  }

  energy_change /= NUM_VALUES_TO_USE - 1;

  std::cerr << "Current ratio is: " <<  energy_change/energy_values.back() << "\n";
  std::cerr << "Target ratio for convergence is: " << 1.0/1000 << "\n";

  return !(energy_change/energy_values.back() > 1.0/1000);

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
