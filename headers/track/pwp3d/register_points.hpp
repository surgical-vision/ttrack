#ifndef __REGISTER_POINTS_HPP__
#define __REGISTER_POINTS_HPP__
#include "../../headers.hpp"
#include "../pose.hpp"
#include "../../utils/camera.hpp"
#include "../kalman.hpp"

namespace ttrk {

  struct MatchedPair {
    cv::Vec3d image_point;
    cv::Vec3d learned_point;
  };


  struct Descriptor {
    void write(cv::FileStorage &ofs, int count) const{
      cv::Mat t(3,1,CV_64FC1);
      for(int i=0;i<3;i++)
        t.at<double>(i) = coordinate[i];
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

      for(int i=0;i<3;i++){
        if(mcoord.type() != CV_64FC1)
          coordinate[i] = mcoord.at<double>(i);
        else if(mcoord.type() != CV_32FC1)
          coordinate[i] = mcoord.at<float>(i);
      }

      ss2 << "Descriptor" << count;
      ifs[ss2.str()] >> descriptor;
    }
    cv::Vec3d coordinate;
    cv::Mat descriptor;
    double TEST_DISTANCE;
  }; 

  struct DescriptorMatches{
    Descriptor gt;
    Descriptor left_image;
    Descriptor right_image;
  };


  const int NUM_DESCRIPTOR = 120;
  const int MATCHING_DISTANCE_THRESHOLD = 25;
  const double DESCRIPTOR_SIMILARITY_THRESHOLD = 240.0;

  class PointRegistration {

  public:

    PointRegistration(boost::shared_ptr<MonocularCamera> camera);

    cv::Mat GetPointDerivative(const cv::Point3d &world, cv::Point2d &image, const Pose &pose) const;
    
    void FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, boost::shared_ptr<Model> model, const Pose &pose, cv::Mat &save_image);    
  
    void ComputeDescriptorsForPointTracking(boost::shared_ptr<sv::Frame> frame, KalmanTracker current_model, const cv::Mat &shape_image );

  protected:

    void GetDescriptors(const cv::Mat &frame, std::vector<Descriptor> &ds);
    void MatchDescriptorsToModel(std::vector<Descriptor> &d1, std::vector<Descriptor> &d2, std::vector<DescriptorMatches> &dm);
    void ReadKeypoints(const std::string filename, std::vector<Descriptor> &descriptors, int count);
    
    void FindCorrespondingMatches(std::vector<Descriptor> &right_ds, std::vector<DescriptorMatches> &matched_ds);

    void FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair);
    
  private:

    boost::shared_ptr<MonocularCamera> camera_;

  };


}


#endif
