
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

PointRegistration::PointRegistration(boost::shared_ptr<MonocularCamera> camera)  : camera_(camera) {

  dm_ = cv::DescriptorMatcher::create("L2");

}


void PointRegistration::ComputeDescriptorsForPointTracking(cv::Mat &frame, cv::Mat &point_map, cv::Mat &normal_map, Pose &pose){

  std::vector< FoundKeyPoint > initial_keypoints;

  GetDescriptors(frame, initial_keypoints);

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
    cv::Vec3f front = point_map.at<cv::Vec3f>(std::roundf(pt.x), std::roundf(pt.y));
    cv::Vec3f normal = normal_map.at<cv::Vec3f>(std::roundf(pt.x), std::roundf(pt.y));

    cv::Vec3f front_on_model = pose.InverseTransformPoint(front);
    cv::Vec3f normal_on_model = pose.InverseTransformPoint(normal);

    Descriptor d(front_on_model, normal, descriptors.row(i));
    i++;
    model_points.push_back(d);
    
    //add 
    dm_->add(descriptors.row(i));

  }

  dm_->train();

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
  GetDescriptors(stereo_frame->GetLeftImage(), frame_descriptors);

  //kind of slow to do it again but will optimize later
  cv::Mat source_descriptors(selection.size(), selection[0]->GetDescriptor().total(), selection[0]->GetDescriptor().type());
  cv::Mat target_descriptors(frame_descriptors.size(), source_descriptors.cols, source_descriptors.type());

  for (auto i = 0; i < selection.size(); ++i){
    source_descriptors.row(i) = selection[i]->GetDescriptor();
  }

  for (auto i = 0; i < frame_descriptors.size(); ++i){
    target_descriptors.row(i) = frame_descriptors[i].second;
  }
  
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

void PointRegistration::GetDescriptors(const cv::Mat &frame, std::vector< FoundKeyPoint > &kps){
  
  cv::Mat image_gray;
  cv::cvtColor(frame, image_gray, CV_RGB2GRAY);
  cv::SiftFeatureDetector detector;//(400);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image_gray, keypoints);
  cv::SiftDescriptorExtractor extractor;
  cv::Mat descriptors_image;
  extractor.compute(image_gray, keypoints, descriptors_image);

  int i = 0;
  for (auto kp = keypoints.begin(); kp != keypoints.end(); kp++, i++){

    kps.push_back(FoundKeyPoint(*kp, descriptors_image.row(i)));

  }

}