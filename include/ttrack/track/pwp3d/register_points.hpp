#ifndef __REGISTER_POINTS_HPP__
#define __REGISTER_POINTS_HPP__
#include "../../headers.hpp"
#include "../pose.hpp"
#include "../../utils/camera.hpp"
#include "../../track/model/model.hpp"

namespace ttrk {

  struct MatchedPair {
    cv::Vec3d image_point;
    cv::Vec3d learned_point;
  };
  
  struct Descriptor {
    cv::Vec3f coordinate;
    cv::Mat descriptor;
    double TEST_DISTANCE;
  }; 

  struct DescriptorMatches{
    Descriptor gt;
    Descriptor left_image;
    Descriptor right_image;
  };


  const int NUM_DESCRIPTOR = 120;
  const int MATCHING_DISTANCE_THRESHOLD = 10;
  const double DESCRIPTOR_SIMILARITY_THRESHOLD = 200.0;

  class PointRegistration {

  public:

    PointRegistration(boost::shared_ptr<MonocularCamera> camera);

    std::vector<float> GetPointDerivative(const cv::Point3d &world, cv::Point2f &image, const Pose &pose) ;
    
    void FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, boost::shared_ptr<Model> model, const Pose &pose, std::vector<MatchedPair> &pnp);    
  
    void ComputeDescriptorsForPointTracking(cv::Mat &frame, cv::Mat &point_map, const Pose &pose); 
  
    float average_distance;
    
  protected:

    void GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds);
    void MatchDescriptorsToModel(std::vector<Descriptor> &d1, std::vector<Descriptor> &d2, std::vector<DescriptorMatches> &dm);
    void ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count);
    
    void FindCorrespondingMatches(std::vector<Descriptor> &right_ds, std::vector<DescriptorMatches> &matched_ds);

    void FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair);
    

  private:

    boost::shared_ptr<MonocularCamera> camera_;

    std::vector<Descriptor> model_points;

  };


}


#endif
