#include "../../../headers/track/pwp3d/stereo_pwp3d.hpp"
#include "../../../headers/utils/helpers.hpp"
#include<boost/filesystem.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace ttrk;
#define SAVEDEBUG

/*** REMOVE THIS ***/


void StereoPWP3D::DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const {

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



Pose StereoPWP3D::TrackTargetInFrame(KalmanTracker current_model, boost::shared_ptr<sv::Frame> frame){
  

  frame_ = frame;
  const int NUM_STEPS = 25;
  cv::Vec3f initial_translation = current_model.CurrentPose().translation_;

#ifdef DEBUG
    boost::progress_timer t; //timer prints time when it goes out of scope
#endif


#ifdef SAVEDEBUG
  std::cerr << "starting pose is: "<< current_model.CurrentPose().rotation_ << " + " << cv::Point3f(current_model.CurrentPose().translation_) << std::endl;
  cv::Mat canvas = frame_->Mat().clone();
  DrawModelOnFrame(current_model,canvas);
  static int frame_count = 0;
  std::stringstream ss; ss << "frame_" << frame_count;
  boost::filesystem::create_directory(ss.str());
  cv::imwrite(ss.str()+"/step_init.png",canvas);
  frame_count++;
  
  DEBUG_DIR_ = ss.str() + "/debug/";
  boost::filesystem::create_directory(DEBUG_DIR_);
  std::ofstream ENERGY_FILE((DEBUG_DIR_ + "/energy_file.csv").c_str());
  if(!ENERGY_FILE.is_open()) throw(std::runtime_error("Error, could not open energy file!\n"));
#endif

  //ComputeDescriptorsForPointTracking(frame_,current_model);

  //store a vector of Pose values. check std. dev. of these values, if this is small enough, assume convergence.
  std::vector<Pose> convergence_test_values;
  bool converged = false;

  //values to hold the 'best' pwp3d estimate
  double min_energy = std::numeric_limits<double>::max();
  Pose pwp3d_best_pose = current_model.CurrentPose();
  
  //iterate until convergence
  for(int step=0; step < NUM_STEPS && !converged; step++){

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

    //(x,y,z,w,r1,r2,r3)
    cv::Mat jacobian = cv::Mat::zeros(7,1,CV_64FC1);
    boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
    
    //setup values to get the
    double energy = 0.0;
    

    for(int r=0;r<ROI_left_.rows;r++){
      for(int c=0;c<ROI_left_.cols;c++){

        if(!stereo_frame->InsideRectifiedRegion(r,c)) continue;
        
        //compute the energy value for this pixel - not used for pose jacobian, just for assessing minima/progress
        energy += GetEnergy(r,c,sdf_image.at<float>(r,c), norm_foreground, norm_background);

        //P_f - P_b / (H * P_f + (1 - H) * P_b)
        const double region_agreement = GetRegionAgreement(r, c, sdf_image.at<float>(r,c), norm_foreground, norm_background);
        
        //dH / dL
        const cv::Mat pose_derivatives = GetPoseDerivatives(r, c, sdf_image, dSDFdx.at<float>(r,c), dSDFdy.at<float>(r,c), current_model);
      
        //find the stereo regularized depth
        const cv::Mat regularized_depth = GetRegularizedDepth(r,c,current_model);

        //update the jacobian
        for(int i=0;i<pose_derivatives.rows;i++){
          double pp = (region_agreement*pose_derivatives.at<double>(i,0));
          if (pp != pp)
            continue;
          jacobian.at<double>(i,0) += -1 * (region_agreement*pose_derivatives.at<double>(i,0)) + regularized_depth.at<double>(i,0);
        }
       
      }
    }

#ifdef SAVEDEBUG
    ENERGY_FILE << energy << " ";
    ENERGY_FILE.flush();
    //std::cout << "ENERGY IS : " << energy << std::endl;
#endif
    
    if(energy < min_energy) {
      std::cerr << "New minimum energy at step " << step << "\n";
      std::cerr << "Energy is " << energy << "\n";
      min_energy = energy;
      pwp3d_best_pose = current_model.CurrentPose();
    }
    //else break;

    ScaleJacobian(jacobian,step);
    ApplyGradientDescentStep(jacobian,current_model.CurrentPose());
    

#ifdef SAVEDEBUG
    cv::Mat canvas = frame_->Mat().clone();
    DrawModelOnFrame(current_model,canvas);
    std::stringstream ss_2; ss_2 << "step_" << step << ".png";
    cv::imwrite(ss.str()+"/"+ss_2.str(),canvas);
#endif

  }

#ifdef SAVEDEBUG
  ENERGY_FILE.close();
#endif
  
  
  /********************************************************************/

  Pose point_registration_pose = ApplyPointBasedRegistration(frame,current_model);
  
  double point_energy = 0.0;
  {
    current_model.CurrentPose() = p;
    cv::Mat sdf_image = ProjectShapeToSDF(current_model);

    //compute the normalization values n_f and n_b
    double norm_foreground,norm_background;
    ComputeNormalization(norm_foreground,norm_background,sdf_image);
    if(norm_foreground == 0) {
      std::cerr << "Error, the point tracking gave a terrible result!\n";
      return pwp3d_pose;
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

  if (min_energy < point_energy){
    
    current_model.CurrentPose() = pwp3d_best_pose;
    
    std::cerr << "MIN_ENERGY = " << min_energy << "\n";
    std::cerr << "POINT ENERGY = " << point_energy << "\n";
    std::cerr << "Returning pwp3d energy!\n";
    
  }else{
    
    current_model.CurrentPose() = point_registration_pose;

    std::cerr << "MIN_ENERGY = " << min_energy << "\n";
    std::cerr << "POINT ENERGY = " << point_energy << "\n";
    std::cerr << "Retuing point energy!\n";
    
  }

  /**********************************************************************/
  
  //update the velocity model... a bit crude
  cv::Vec3f translational_velocity = current_model.CurrentPose().translation_ - initial_translation;
  current_model.CurrentPose().translational_velocity_ = translational_velocity;
  return current_model.CurrentPose();

}


cv::Mat StereoPWP3D::GetRegularizedDepth(const int r, const int c, const KalmanTracker &current_model) const {
  
  const int NUM_DERIVS = 7;
  return cv::Mat::zeros(7,1,CV_64FC1);
  
  /*cv::Vec3f front_intersection;
  cv::Vec3f back_intersection;

  //find the (x,y,z) coordinates of the front and back intersection between the ray from the current pixel and the target object. return zero vector for no intersection.
  GetTargetIntersections(r,c,front_intersection,back_intersection,current_model);
  if(front_intersection == cv::Vec3f(0,0,0)) return cv::Mat::zeros(NUM_DERIVS,1,CV_64FC1);
  
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame_);
  const double z_estimate = front_intersection[2];
  const double z_stereo = stereo_frame->PtrToPointCloud()->at<cv::Vec3f>(r,c)[2]; //scaled by 16?

  const double z_diff = z_estimate - z_stereo;

  //if(z_diff < 1000) {
  //  std::cerr << "estimate = " << z_estimate << "\nstereo = " << z_stereo << "\n\n (" << r << "," << c << ")\n\n\n\n";
  //}
  const double d_mag = 2*(z_estimate - z_stereo)/std::abs(z_estimate - z_stereo);

  cv::Mat x(NUM_DERIVS,1,CV_64FC1);

  for(int dof=0;dof<NUM_DERIVS;dof++){

    //const cv::Vec3f dof_derivatives = GetDOFDerivatives(dof,current_model.CurrentPose(),front_intersection);

    if(dof != 2) x.at<double>(dof,0) = 0;
    else x.at<double>(dof,0) = d_mag ;//* dof_derivatives[2];

  }

  return x;*/
  
}




void StereoPWP3D::FindROI(const std::vector<cv::Vec2i> &convex_hull) {

  ROI() = frame_->Mat(); //UNTIL I DO THIS FUNCTION

  /*
  for(int r=0;r<frame_->rows();r++){
    for(int c=0;c<frame_->cols();c++){



    }
  }*/
  
}

class MatchedPair {
public:
  cv::Vec3f image_point;
  cv::Vec3f learned_point;
};

class Descriptor {
  public:
  void write(cv::FileStorage &ofs, int count) const{
    cv::Mat t(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
      t.at<float>(i) = coordinate[i];
    std::stringstream ss1,ss2;
    ss1 << "Coordinate" << count;
    ss2 << "Descriptor" << count;
    ofs << ss1.str() << t << ss2.str() << descriptor;
  }
  void read(cv::FileStorage &ifs, int count) {
    std::stringstream ss1,ss2;
    ss1 << "Coordinate" << count;
    cv::Mat mcoord;
    ifs[ss1.str()] >> mcoord;
    
    for(int i=0;i<3;i++)
      coordinate[i] = mcoord.at<float>(i);
    
    ss2 << "Descriptor" << count;
    ifs[ss2.str()] >> descriptor;
  }
  cv::Vec3f coordinate;
  cv::Mat descriptor;
};

class DescriptorMatches{

public:
  Descriptor gt;
  Descriptor left_image;
  Descriptor right_image;
};
void FindTransformationToImagePlane(std::vector<DescriptorMatches> matches,cv::Mat &rotation, cv::Mat &translation,  boost::shared_ptr<StereoCamera> cam, Pose current_pose);
void FindTransformation(std::vector<MatchedPair> &matched_pair, cv::Mat &rotation, cv::Mat &translation);
void GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds);
void MatchDescriptorsToModel(std::vector<Descriptor> &d1, std::vector<Descriptor> &d2, std::vector<DescriptorMatches> &dm,boost::shared_ptr<StereoCamera> cam,Pose &pose,cv::Mat &left);
void FindCorrespondingMatches(std::vector<Descriptor> &right_ds, std::vector<DescriptorMatches> &matched_ds);
void TriangulateMatches(std::vector<DescriptorMatches> &matches,std::vector<MatchedPair> &matched_3d_points, boost::shared_ptr<StereoCamera> cam, cv::Mat &left, cv::Mat &right);
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

#define NUM_DESCRIPTOR 60

Pose StereoPWP3D::ApplyPointBasedRegistration(boost::shared_ptr<sv::Frame> frame, KalmanTracker &current_model ){

  
  std::vector<Descriptor> ds;
  ReadKeypoints("Keypoints.xml",ds,NUM_DESCRIPTOR); 
  cv::Mat descriptors;
  for(auto d = ds.begin(); d != ds.end() ; d++){
    descriptors.push_back(d->descriptor);
  }
  
  std::vector<Descriptor> left_ds;
  boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);
  GetDescriptors( stereo_frame->LeftMat() , left_ds);
  std::vector<DescriptorMatches> matches;
  MatchDescriptorsToModel(ds,left_ds,matches,stereo_camera_,current_model.CurrentPose(),stereo_frame->LeftMat());

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

void MatchDescriptorsToModel(std::vector<Descriptor> &model_descriptors, std::vector<Descriptor> &image_descriptors, std::vector<DescriptorMatches> &found_matches, boost::shared_ptr<StereoCamera> cam, Pose &pose, cv::Mat &left){
  
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
    cv::cvtColor(frame_->Mat(),image_gray,CV_RGB2GRAY);
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
