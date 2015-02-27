
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <numeric>

#include "../../../include/ttrack/track/pwp3d/register_points.hpp"
#include "../../../include/ttrack/utils/helpers.hpp"
#include "../../../include/ttrack/track/pwp3d/pwp3d.hpp"
#include "../../../include/ttrack/constants.hpp"

using namespace ttrk;


#ifndef OLD_POINT_REGISTER

PointRegistration::PointRegistration(boost::shared_ptr<MonocularCamera> camera) : camera_(camera) {



}


void PointRegistration::ComputeDescriptorsForPointTracking(cv::Mat &frame, cv::Mat &point_map, const Pose &pose){

  //make a descriptor finder
  cv::Mat image_gray;
  cv::cvtColor(frame, image_gray, CV_RGB2GRAY);

  //model_points.clear();

  cv::SiftFeatureDetector detector;//(400);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image_gray, keypoints);
  std::vector<cv::KeyPoint> tool_keypoints;

  //find the keypoints and filter for those inside the model projection
  for (auto kp = keypoints.begin(); kp != keypoints.end(); kp++){

    cv::Point2f &pt = kp->pt;
    if (point_map.at<cv::Vec3f>(pt.y, pt.x) != cv::Vec3f((float)GL_FAR, (float)GL_FAR, (float)GL_FAR)){
      tool_keypoints.push_back(*kp);
    }

  }

  //sort them by their responses
  std::sort(tool_keypoints.begin(), tool_keypoints.end(),
    [](const cv::KeyPoint &a, const cv::KeyPoint &b) -> bool {
    return a.response > b.response;
  });

  //collect descriptors in the image plane
  cv::SiftDescriptorExtractor extractor;
  cv::Mat descriptors;
  extractor.compute(image_gray, tool_keypoints, descriptors);
  int i = 0;

  //only keep the NUM_DESCRIPTOR best descriptors
  if (tool_keypoints.size() > NUM_DESCRIPTOR){
    tool_keypoints = std::vector<cv::KeyPoint>(tool_keypoints.begin(), tool_keypoints.begin() + NUM_DESCRIPTOR);
  }

  //project them onto the model
  for (auto kp = tool_keypoints.begin(); kp != tool_keypoints.end(); kp++){

    cv::Point2f &pt = kp->pt;
    cv::Vec3f front = point_map.at<cv::Vec3f>(cv::Point2i(int(pt.x), int(pt.y)));

    cv::Vec3f front_on_model = pose.InverseTransformPoint(front);

    Descriptor d;
    d.coordinate = front_on_model;
    d.descriptor = descriptors.row(i);
    i++;
    model_points.push_back(d);

  }

  int x = 0;
  //previous_frame = frame->GetImageROI().clone();

}



void PointRegistration::FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, boost::shared_ptr<Model> model, const Pose &pose, std::vector<MatchedPair> &pnp){

  pnp.clear();
  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);

  //transform the points from the base reference frame into the reference frame of the current camera
  std::vector<Descriptor> current_frame;
  for (auto i = 0; i < model_points.size(); ++i){
    current_frame.push_back(Descriptor());
    current_frame[i].coordinate = pose.TransformPoint(model_points[i].coordinate);
    current_frame[i].descriptor = model_points[i].descriptor;
    current_frame[i].TEST_DISTANCE = model_points[i].TEST_DISTANCE;
  }


  //search the image plane for features to match
  std::vector<Descriptor> frame_descriptors;
  GetDescriptors(stereo_frame->GetLeftImage(), frame_descriptors);

  //for each keypoint
  for (auto kp = current_frame.begin(); kp != current_frame.end(); kp++){

    std::vector<std::pair<Descriptor, double> > matching_queue;
    //project them to the image plane
    cv::Point2d projected_pt = camera_->ProjectPointToPixel(cv::Point3d(kp->coordinate));

    //iterate over the found features
    for (auto frame_descriptor = frame_descriptors.begin(); frame_descriptor != frame_descriptors.end(); frame_descriptor++){

      cv::Point2d pt_to_match(frame_descriptor->coordinate[0], frame_descriptor->coordinate[1]);

      //if the euclidean distance is < threshold then add this point to matching vector
      double euclidean_distance = std::sqrt((projected_pt.x - pt_to_match.x)*(projected_pt.x - pt_to_match.x) + (projected_pt.y - pt_to_match.y)*(projected_pt.y - pt_to_match.y));
      if (euclidean_distance < MATCHING_DISTANCE_THRESHOLD) {
        matching_queue.push_back(std::pair<Descriptor, double>(*frame_descriptor, 0.0));
        matching_queue.back().first.TEST_DISTANCE = euclidean_distance;
      }

    }

    if (!matching_queue.size()) continue; //no matches found :(

    for (auto mq = matching_queue.begin(); mq != matching_queue.end(); mq++){

      mq->second = l2_norm(kp->descriptor, mq->first.descriptor);

    }

    std::sort(matching_queue.begin(), matching_queue.end(), [](const std::pair<Descriptor, double>& before, const std::pair<Descriptor, double>& after) -> bool
    {
      return before.second < after.second;
    });

    //run l2 norm based matching between learned point and points in this vector. is the matching score is good enough, add to the matches

    double size_of_best = matching_queue.front().second;//l2_norm( matching_queue.front().first.descriptor, cv::Mat::zeros(matching_queue.front().first.descriptor.size(),matching_queue.front().first.descriptor.type()));  

    if ((matching_queue.size() > 1 && size_of_best / matching_queue[1].second < 0.8) || (matching_queue.size() == 1 && size_of_best < DESCRIPTOR_SIMILARITY_THRESHOLD)){

      MatchedPair mp;
      mp.learned_point = kp->coordinate;
      mp.image_point = matching_queue.front().first.coordinate;
      pnp.push_back(mp);

    }

  }

}

std::vector<float> PointRegistration::GetPointDerivative(const cv::Point3d &world, cv::Point2f &image, const Pose &pose) {

  cv::Vec3d front_intersection(world);

  if (front_intersection[2] == 0.0) front_intersection[2] = 0.001;
  double z_inv_sq = 1.0 / front_intersection[2];

  cv::Point2d projected_world = camera_->ProjectPointToPixel(world);

  ci::Vec3f world_n(world.x, world.y, world.z);
  std::vector<ci::Vec3f> jacs = pose.ComputeJacobian(world_n);

  std::vector<float> jacobians;

  const double x_error = image.x - projected_world.x;
  const double y_error = image.y - projected_world.y;

  for (int dof = 0; dof<pose.GetNumDofs(); dof++){

    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const double dXdL = camera_->Fx() * (z_inv_sq*((front_intersection[2] * dof_derivatives[0]) - (front_intersection[0] * dof_derivatives[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq*((front_intersection[2] * dof_derivatives[1]) - (front_intersection[1] * dof_derivatives[2])));
    //jacobians.push_back(2*((image.x - projected_world.x)*dXdL + (image.y - projected_world.y)*dYdL)); 
    const double inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
    const double dPxdL = -2 * x_error * dXdL;
    const double dPydL = -2 * y_error * dYdL;
    jacobians.push_back(inv_sqrt * (dPxdL + dPydL));

  }

  return jacobians;

}


void PointRegistration::ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count){

  //cv::FileStorage ifs(filename, cv::FileStorage::READ);
  //if(!ifs.isOpened()) {
  //  throw(std::runtime_error("ERror could not open file!\n"));
  //}

  //for(int n=0; n<count;n++){

  //  try{
  //    Descriptor ds;
  //    ds.read(ifs,n);
  //    descriptors.push_back(ds);
  //  }catch(...){
  //    std::cerr << "Error, not enough KeyPoints\n";
  //    break;
  //  }

  //}

  //if (descriptors.size() < 4) {
  //  throw(std::runtime_error("Error, could not find more than 3 good descriptors in file!\n"));
  //}
}



void PointRegistration::FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair){

  throw(std::runtime_error("Error, not implemented!\n"));
  /*
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
  */

}




void PointRegistration::GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds){
  cv::Mat image_gray;
  cv::cvtColor(frame, image_gray, CV_RGB2GRAY);
  cv::SiftFeatureDetector detector;//(400);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image_gray, keypoints);
  cv::SiftDescriptorExtractor extractor;
  cv::Mat descriptors_image;
  extractor.compute(image_gray, keypoints, descriptors_image);
  int i = 0;
  for (auto kp = keypoints.begin(); kp != keypoints.end(); kp++){
    Descriptor d;
    d.coordinate = cv::Vec3d(kp->pt.x, kp->pt.y, 0);
    d.descriptor = descriptors_image.row(i);
    i++;
    ds.push_back(d);
  }
}

void PointRegistration::MatchDescriptorsToModel(std::vector<Descriptor> &model_descriptors, std::vector<Descriptor> &image_descriptors, std::vector<DescriptorMatches> &found_matches){

  cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector<cv::DMatch> matches;
  cv::Mat model_descriptor_matrix, image_descriptor_matrix;

  for (auto d = model_descriptors.begin(); d != model_descriptors.end(); d++)
    model_descriptor_matrix.push_back(d->descriptor);

  for (auto d = image_descriptors.begin(); d != image_descriptors.end(); d++)
    image_descriptor_matrix.push_back(d->descriptor);

  /* match( queryMatrix, traingMatrix ) */
  matcher.match(model_descriptor_matrix, image_descriptor_matrix, matches);

  for (auto match = matches.begin(); match != matches.end(); match++){

    DescriptorMatches d;

    d.gt = model_descriptors[match->queryIdx];
    d.left_image = image_descriptors[match->trainIdx];
    d.right_image.coordinate = cv::Vec3d(0, 0, 0);
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
  cv::Mat right_image_descriptor_matrix, left_image_descriptor_matrix;
  for (auto d = right_image_descriptors.begin(); d != right_image_descriptors.end(); d++)
    right_image_descriptor_matrix.push_back(d->descriptor);
  for (auto d = left_image_matches.begin(); d != left_image_matches.end(); d++)
    left_image_descriptor_matrix.push_back(d->left_image.descriptor);

  /* match( queryMatrix, traingMatrix ) */
  matcher.match(right_image_descriptor_matrix, left_image_descriptor_matrix, matches);

  std::sort(matches.begin(), matches.end(),
    [](const cv::DMatch &a, const cv::DMatch &b) -> bool {
    return a.distance < b.distance;
  });

  matches.erase(matches.begin() + 20, matches.end());

  for (auto match = matches.begin(); match != matches.end(); match++){

    DescriptorMatches &dm = left_image_matches[match->trainIdx];
    Descriptor &d = right_image_descriptors[match->queryIdx];
    dm.right_image = d;

  }

}



#else

PointRegistration::PointRegistration(boost::shared_ptr<MonocularCamera> camera)  : camera_(camera) {

  dm_ = cv::DescriptorMatcher::create("BruteForce"); //BruteForce uses L2 norm

}


void PointRegistration::ComputeDescriptorsForPointTracking(cv::Mat &frame, cv::Mat &point_map, cv::Mat &normal_map, Pose &pose){

  std::vector< FoundKeyPoint > initial_keypoints;

  GetDescriptors(frame, point_map, initial_keypoints);

  int i = 0;
  //project them onto the model
  for (auto kp = initial_keypoints.begin(); kp != initial_keypoints.end(); kp++){

    cv::Point2f &pt = kp->first.pt;
    cv::Vec3f front = point_map.at<cv::Vec3f>(std::roundf(pt.y), std::roundf(pt.x));
    //cv::Vec3f normal = normal_map.at<cv::Vec3f>(std::roundf(pt.y), std::roundf(pt.x));

    cv::Vec3f front_on_model = pose.InverseTransformPoint(front);
    //cv::Vec3f normal_on_model = pose.InverseTransformPoint(normal);
    cv::Vec3f normal(0, 0, 0);

    Descriptor d(front_on_model, normal, kp->second.row(i));
    i++;
    model_points.push_back(d);
    
    //add 
    //dm_->add(descriptors.row(i));

  }

  //dm_->train();

}



void PointRegistration::FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, boost::shared_ptr<Model> model, const Pose &pose, std::vector<MatchedPair> &pnp){

  pnp.clear();

  auto stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(frame);

  //transform the points from the base reference frame into the reference frame of the current camera
  std::vector<Descriptor *> selection;
  for (auto i = 0; i < model_points.size(); ++i){

    if (!model_points[i].ShouldUseThisDescriptor(model->GetBasePose())){
      continue;
    }

    selection.push_back(&model_points[i]);

  }

  if (selection.size() == 0) throw std::runtime_error("Error, throwing away all the points");

  //search the image plane for features to match
  std::vector<FoundKeyPoint> frame_descriptors;
  GetDescriptors(stereo_frame->GetLeftImage(), front_intersection_image, frame_descriptors);

  //kind of slow to do it again but will optimize later
  cv::Mat source_descriptors(selection.size(), selection[0]->GetDescriptor().total(), selection[0]->GetDescriptor().type());
  cv::Mat target_descriptors(frame_descriptors.size(), source_descriptors.cols, source_descriptors.type());

  for (auto i = 0; i < selection.size(); ++i){
    cv::Mat &d = selection[i]->GetDescriptor();
    for (int c = 0; c < d.cols; ++c){
      source_descriptors.at<float>(i, c) = d.at<float>(c);
    }
  }

  for (auto i = 0; i < frame_descriptors.size(); ++i){
    cv::Mat &d = frame_descriptors[i].second;
    for (int c = 0; c < d.cols; ++c){
      target_descriptors.at<float>(i, c) = d.at<float>(c);
    }
  }

  //find best match for each descriptor from argument 1 (queryDescriptors) by searching argument 2 (trainDescriptors)
  std::vector<cv::DMatch> matches;
  dm_->match(target_descriptors, source_descriptors, matches);
  
  for (auto i = 0; i < matches.size(); ++i){
    
    cv::DMatch &mi = matches[i];

    MatchedPair mp;
    mp.eye_coordinates = selection[mi.trainIdx]->cvGetPointInEyeSpace(model->GetBasePose());
    mp.image_coordinates = frame_descriptors[mi.queryIdx].first.pt;
    cv::Point2f predicted_projection = camera_->ProjectPoint(cv::Point3f(mp.eye_coordinates));

    selection[mi.trainIdx]->IncrementTotalNumberOfAttempts();

    

    if (l2_distance(cv::Vec2f(predicted_projection), cv::Vec2f(mp.image_coordinates)) > 20){
      continue;
    }
    
    selection[mi.trainIdx]->IncrementNumberOfMatches();
    mp.matching_distance = mi.distance;
    
    pnp.push_back(mp);

  }



}

std::vector<float> PointRegistration::GetPointDerivative(const cv::Point3d &world, cv::Point2f &image, const Pose &pose) {
    
  cv::Vec3d front_intersection(world);

  if(front_intersection[2] == 0.0) front_intersection[2] = 0.001;
  double z_inv_sq = 1.0/front_intersection[2];

  cv::Point2d projected_world = camera_->ProjectPointToPixel(world);

  ci::Vec3f world_n(world.x, world.y, world.z);
  std::vector<ci::Vec3f> jacs = pose.ComputeJacobian(world_n);

  std::vector<float> jacobians;

  const double x_error = image.x - projected_world.x;
  const double y_error = image.y - projected_world.y;

  for(int dof=0;dof<pose.GetNumDofs();dof++){

    const ci::Vec3f ddi = jacs[dof];
    const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

    const double dXdL = camera_->Fx() * (z_inv_sq*((front_intersection[2]*dof_derivatives[0]) - (front_intersection[0]*dof_derivatives[2])));
    const double dYdL = camera_->Fy() * (z_inv_sq*((front_intersection[2]*dof_derivatives[1]) - (front_intersection[1]*dof_derivatives[2])));
    //jacobians.push_back(2*((image.x - projected_world.x)*dXdL + (image.y - projected_world.y)*dYdL)); 
    const double inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
    const double dPxdL = -2 * x_error * dXdL;
    const double dPydL = -2 * y_error * dYdL;
    jacobians.push_back(inv_sqrt * (dPxdL + dPydL));

  }

  return jacobians;

}

void PointRegistration::GetDescriptors(const cv::Mat &frame, const cv::Mat &filter, std::vector< FoundKeyPoint > &kps){
  
  cv::Mat image_gray;
  cv::cvtColor(frame, image_gray, CV_RGB2GRAY);
  cv::SiftFeatureDetector detector;//(400);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image_gray, keypoints);
  cv::SiftDescriptorExtractor extractor;
  cv::Mat descriptors_image;
  extractor.compute(image_gray, keypoints, descriptors_image);

  std::vector<cv::KeyPoint> tool_keypoints;

  int i = 0;
  for (auto kp = keypoints.begin(); kp != keypoints.end(); kp++, i++){

    if (filter.empty()){
      kps.push_back(FoundKeyPoint(*kp, descriptors_image.row(i)));
    }

    else if (filter.at<cv::Vec3f>(kp->pt.y, kp->pt.x) != cv::Vec3f((float)GL_FAR, (float)GL_FAR, (float)GL_FAR)){
      kps.push_back(FoundKeyPoint(*kp, descriptors_image.row(i)));
    }

  }

}

/*

//make a descriptor finder
cv::Mat image_gray;
cv::cvtColor(frame, image_gray, CV_RGB2GRAY);

model_points.clear();

cv::SiftFeatureDetector detector;//(400);
std::vector<cv::KeyPoint> keypoints;
detector.detect(image_gray, keypoints);
std::vector<cv::KeyPoint> tool_keypoints;

//find the keypoints and filter for those inside the model projection
for (auto kp = keypoints.begin(); kp != keypoints.end(); kp++){

cv::Point2f &pt = kp->pt;
if (point_map.at<cv::Vec3f>(pt.y, pt.x) != cv::Vec3f((float)GL_FAR, (float)GL_FAR, (float)GL_FAR)){
tool_keypoints.push_back(*kp);
}

}

//sort them by their responses
std::sort(tool_keypoints.begin(), tool_keypoints.end(),
[](const cv::KeyPoint &a, const cv::KeyPoint &b) -> bool {
return a.response > b.response;
});

//collect descriptors in the image plane
cv::SiftDescriptorExtractor extractor;
cv::Mat descriptors;
extractor.compute(image_gray, tool_keypoints, descriptors);
int i = 0;

//only keep the NUM_DESCRIPTOR best descriptors
//if (tool_keypoints.size() > NUM_DESCRIPTOR){
//  tool_keypoints = std::vector<cv::KeyPoint>(tool_keypoints.begin(), tool_keypoints.begin() + NUM_DESCRIPTOR);
//}


*/

#endif