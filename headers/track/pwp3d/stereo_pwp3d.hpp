#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {

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
 



void FindTransformation(std::vector<MatchedPair> &matched_pair, cv::Mat &rotation, cv::Mat &translation);

void FindCorrespondingMatches(std::vector<Descriptor> &right_ds, std::vector<DescriptorMatches> &matched_ds);
void TriangulateMatches(std::vector<DescriptorMatches> &matches,std::vector<MatchedPair> &matched_3d_points, boost::shared_ptr<StereoCamera> cam, cv::Mat &left, cv::Mat &right);


  class StereoPWP3D : public PWP3D {

  public: 

    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);
    boost::shared_ptr<StereoCamera> &GetStereoCamera() { return stereo_camera_; } //* Probably this shouldn't be a reference. Change it so it is not. */

  protected:   

    void DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) ;
    void DrawModelOnBothFrames(const KalmanTracker &tracked_model, cv::Mat left_canvas, cv::Mat right_canvas);

    Pose ApplyPointBasedRegistration(boost::shared_ptr<sv::Frame> frame, KalmanTracker &current_model );
    void FindPointCorrespondences(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &matched_pair);
    void FindPointCorrespondencesWithPose(boost::shared_ptr<sv::Frame> frame, std::vector<MatchedPair> &pnp, const Pose &pose);    

    cv::Mat GetPointDerivative(const cv::Point3f &world, cv::Point2f &image, const Pose &pose) const;

    void ComputeDescriptorsForPointTracking(boost::shared_ptr<sv::Frame> frame, KalmanTracker current_model );
    //bool HasGradientDescentConverged(std::vector<Pose> &convergence_test_values, Pose &current_estimate) const;
    bool HasGradientDescentConverged(const cv::Mat &jacobian, const Pose &pose) const ;
    bool HasGradientDescentConverged_UsingEnergy(std::vector<double> &energy_values) const ;
    bool HasGradientDescentConverged__new(std::vector<cv::Mat> &convergence_test_values, cv::Mat &current_estimate) const;
    cv::Vec3f GetDOFDerivativesRightEye(const int dof, const Pose &pose, const cv::Vec3f &point_) ;
    cv::Mat StereoPWP3D::GetPoseDerivativesRightEye(const int r, const int c, const cv::Mat &sdf, const float dSDFdx, const float dSDFdy, KalmanTracker &current_model);
    
    bool SetupEye(const int eye, Pose &pose);

    boost::shared_ptr<StereoCamera> stereo_camera_;
    //cv::Mat ROI_left_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    //cv::Mat ROI_right_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };


}

#endif
