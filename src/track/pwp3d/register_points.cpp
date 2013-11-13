#include "../../../headers/track/pwp3d/register_points.hpp"
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <numeric>
#include "../../../headers/utils/helpers.hpp"
#include "../../../headers/track/pwp3d/pwp3d.hpp"

using namespace ttrk;

void PointRegistration::FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &pnp, const Pose &pose){

  pnp.clear();

  //load the ground truth points from the file
  std::vector<Descriptor> ground_truth_descriptors;
  ReadKeypoints(config_dir_ + "/Keypoints.xml",ground_truth_descriptors,NUM_DESCRIPTOR); 
  //transform them to the current coordinate system
  for(auto kp = ground_truth_descriptors.begin(); kp != ground_truth_descriptors.end(); kp++){

    kp->coordinate = pose.Transform(kp->coordinate);

  }

  double average_distance = 0.0;

  //search the image plane for features to match
  std::vector<Descriptor> frame_descriptors;
  GetDescriptors(frame->GetImageROI(),frame_descriptors);

  //for each keypoint
  for(auto kp=ground_truth_descriptors.begin(); kp != ground_truth_descriptors.end(); kp++){

    std::vector<std::pair<Descriptor, double> > matching_queue;
    //project them to the image plane
    cv::Point2f projected_pt = camera_->ProjectPointToPixel(cv::Point3f(kp->coordinate));
    //cv::circle(frame->GetImageROI(),cv::Point(projected_pt),3,cv::Scalar(255,12,52),2);

    //iterate over the found features
    for(auto frame_descriptor = frame_descriptors.begin(); frame_descriptor != frame_descriptors.end(); frame_descriptor++){

      cv::Point2f pt_to_match(frame_descriptor->coordinate[0],frame_descriptor->coordinate[1]);

      //if the euclidean distance is < threshold then add this point to matching vector
      double euclidean_distance = std::sqrt((projected_pt.x - pt_to_match.x)*(projected_pt.x - pt_to_match.x) + (projected_pt.y - pt_to_match.y)*(projected_pt.y - pt_to_match.y));
      if(euclidean_distance < MATCHING_DISTANCE_THRESHOLD) {
        matching_queue.push_back(std::pair<Descriptor,double>(*frame_descriptor,0.0));
        matching_queue.back().first.TEST_DISTANCE = euclidean_distance;
      }

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

      average_distance += matching_queue.front().first.TEST_DISTANCE;

      cv::Point2f pt_to_match(matching_queue.front().first.coordinate[0],matching_queue.front().first.coordinate[1]);

      //cv::line(frame->GetImageROI(),pt_to_match,projected_pt,cv::Scalar(244,0,10),3);
      //cv::circle(frame->GetImageROI(),pt_to_match,4,cv::Scalar(24,241,52),2);

      MatchedPair mp;
      mp.learned_point = kp->coordinate;
      mp.image_point = matching_queue.front().first.coordinate;
      pnp.push_back( mp );
    }

  }

  if(pnp.size())
    std::cout << "The average distance is " << average_distance/pnp.size() << "\n";


}

cv::Mat PointRegistration::GetPointDerivative(const cv::Point3f &world, cv::Point2f &image, const Pose &pose) const{

  const int NUM_DERIVS = 7;
  cv::Mat ret(NUM_DERIVS,1,CV_64FC1);
  cv::Vec3f front_intersection(world);

  if(front_intersection[2] == 0.0) front_intersection[2] = 0.001;
  double z_inv_sq = 1.0/front_intersection[2];

  cv::Point2f projected_world = camera_->ProjectPointToPixel(world);

  for(int dof=0;dof<NUM_DERIVS;dof++){

    const cv::Vec3f dof_derivatives = pose.GetDOFDerivatives(dof,cv::Vec3f(world));

    const double dXdL = camera_->Fx() * (z_inv_sq*((front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq*((front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2])));
    ret.at<double>(dof,0) = -2*((image.x - projected_world.x)*dXdL + (image.y - projected_world.y)*dYdL);

  }

  return ret;



}


void PointRegistration::ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count){

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



void PointRegistration::FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair){

  std::vector<Descriptor> ds;
  ReadKeypoints(config_dir_ + "/Keypoints.xml",ds,NUM_DESCRIPTOR); 
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



Pose PointRegistration::ApplyPointBasedRegistration(boost::shared_ptr<sv::Frame> frame, KalmanTracker &current_model ){


  std::vector<Descriptor> ds;
  ReadKeypoints(config_dir_ + "/Keypoints.xml",ds,NUM_DESCRIPTOR); 
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
  FindTransformationToImagePlane(matches,rotation,translation,camera_,current_model.CurrentPose());
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation,rotation_matrix);
  Pose pose(translation,sv::Quaternion(rotation_matrix));
  //current_model.CurrentPose() = pose;
  return pose;
}

void PointRegistration::FindTransformationToImagePlane(std::vector<DescriptorMatches> matches,cv::Mat &rotation, cv::Mat &translation,  boost::shared_ptr<MonocularCamera> cam, Pose current_pose){

  std::vector<cv::Point3f> world_points;
  std::vector<cv::Point2f> image_points;

  for(auto pair = matches.begin();pair!=matches.end();pair++){

    world_points.push_back(cv::Point3f(pair->gt.coordinate));
    image_points.push_back(cv::Point2f(pair->left_image.coordinate[0],pair->left_image.coordinate[1]));

  }
  cv::Rodrigues(current_pose.rotation_.AngleAxis(),rotation);
  translation = cv::Mat(1,3,CV_64FC1);
  for(int i=0;i<3;i++) translation.at<double>(i) = current_pose.translation_[i];
  cv::solvePnPRansac(world_points,image_points,cam->intrinsic_params(),cam->distortion_params(),rotation,translation,true);
}

void PointRegistration::FindTransformation(std::vector<MatchedPair> &matched_pair, cv::Mat &rotation, cv::Mat &translation){

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

void PointRegistration::GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds){
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

void PointRegistration::MatchDescriptorsToModel(std::vector<Descriptor> &model_descriptors, std::vector<Descriptor> &image_descriptors, std::vector<DescriptorMatches> &found_matches){

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

void PointRegistration::FindCorrespondingMatches(std::vector<Descriptor> &right_image_descriptors, std::vector<DescriptorMatches> &left_image_matches){

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

void PointRegistration::TriangulateMatches(std::vector<DescriptorMatches> &matches, std::vector<MatchedPair> &matched_pair, boost::shared_ptr<StereoCamera> cam, cv::Mat &left, cv::Mat &right){

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
  //cv::imwrite("BOTH.png",both);

}

void PointRegistration::ComputeDescriptorsForPointTracking(boost::shared_ptr<sv::Frame> frame, KalmanTracker current_model , const cv::Mat &shape_image ){

  //make a descriptor finder
  cv::Mat image_gray;
  cv::cvtColor(frame->GetImageROI(),image_gray,CV_RGB2GRAY);

  //cv::Mat shape_image;
  //GetSDFAndIntersectionImage(current_model,shape_image,cv::Mat(),cv::Mat());

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

  cv::FileStorage fs(config_dir_ + "/KeyPoints.xml",cv::FileStorage::WRITE);
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

