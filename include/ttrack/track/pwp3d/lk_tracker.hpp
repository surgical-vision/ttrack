#ifndef __LK_TRACKER__
#define __LK_TRACKER__

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../headers.hpp"
#include "../pose.hpp"
#include "../../utils/camera.hpp"
#include "../../track/model/model.hpp"
#include "../../constants.hpp"
namespace ttrk{
  
  struct TrackedPoint {
    TrackedPoint(const cv::Vec3f &mp, const cv::Vec2f &fp) : model_point(mp), frame_point(fp), found_image_point(-1, -1) {}
    cv::Vec3f model_point;
    cv::Vec2f frame_point;
    cv::Vec2f found_image_point;
  };

  

  class LKTracker {

  public:

    LKTracker(boost::shared_ptr<MonocularCamera> camera) : camera_(camera) {
      win_size_ = cv::Size(31, 31);
      term_crit_ = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    }

    void TrackLocalPoints(cv::Mat &current_frame){

      if (previous_frame_gray_.empty() || tracked_points_.size() == 0){
        InitializeTracker(current_frame);
        return; 
      }

      std::vector<unsigned char> status;
      std::vector<float> err;
      cv::cvtColor(current_frame, current_frame_gray_, CV_BGR2GRAY);

      std::vector<cv::Point2f> found_points_in_new_frame;
      cv::calcOpticalFlowPyrLK(previous_frame_gray_, current_frame_gray_, GetPointsOnPreviousImage(), found_points_in_new_frame, status, err, win_size_, 3, term_crit_, 0, 0.001);

      if (tracked_points_.size() != found_points_in_new_frame.size()) throw std::runtime_error("");

      for (size_t i = 0; i < found_points_in_new_frame.size(); ++i){

        if (status[i] == 0){
          tracked_points_[i].found_image_point = cv::Vec2f(-1, -1);
          continue;
        }

        //do something with err?
        tracked_points_[i].found_image_point = found_points_in_new_frame[i];

      }

    }

    std::vector<float> GetDerivativesForPoints(const Pose &pose) {
    
      std::vector<float> ret;
      for (int i = 0; i < 7; ++i){
        ret.push_back(0.0f);
      }

      for (auto tp : tracked_points_){

        std::vector<float> pds = GetPointDerivative(tp.model_point, tp.frame_point, tp.found_image_point, pose);

        for (int i = 0; i < pds.size(); ++i){
          ret[i] += pds[i];
        }

      }

      return ret;
    
    }


    

  protected:

    std::vector<float> GetPointDerivative(const cv::Vec3f &world_previous_, const cv::Vec2f &image_previous, const cv::Vec2f &image_new, const Pose &pose){

      cv::Vec3f world_previous(world_previous_);
      if (world_previous[2] == 0.0) world_previous[2] = 0.001;
      double z_inv_sq = 1.0 / world_previous[2];

      std::vector<ci::Vec3f> jacs = pose.ComputeJacobian(ci::Vec3f(world_previous[0], world_previous[1], world_previous[2]));

      std::vector<float> jacobians;

      const double x_error = image_new[0] - world_previous[0];
      const double y_error = image_new[1] - world_previous[1];

      for (int dof = 0; dof<pose.GetNumDofs(); dof++){

        const ci::Vec3f ddi = jacs[dof];
        const cv::Vec3f dof_derivatives(ddi[0], ddi[1], ddi[2]);

        const double dXdL = camera_->Fx() * (z_inv_sq*((world_previous[2] * dof_derivatives[0]) - (world_previous[0] * dof_derivatives[2])));
        const double dYdL = camera_->Fy() * (z_inv_sq*((world_previous[2] * dof_derivatives[1]) - (world_previous[1] * dof_derivatives[2])));
        const double inv_sqrt = 1.0 / (2.0 * std::sqrt(x_error * x_error + y_error * y_error));
        const double dPxdL = -2 * x_error * dXdL;
        const double dPydL = -2 * y_error * dYdL;
        jacobians.push_back(inv_sqrt * (dPxdL + dPydL));

      }

      return jacobians;


    }

    void InitializeTracker(cv::Mat current_frame){
      const cv::Size subPixWinSize(10, 10);
      
      cv::Mat gray;
      std::vector<cv::Point2f> points;
      cv::goodFeaturesToTrack(gray, points, 20, 0.01, 10, CreateMask(), 3, 0, 0.04);
      cv::cornerSubPix(gray, points, subPixWinSize, cv::Size(-1, -1), term_crit_);
      
      tracked_points_.clear();
      for (size_t i = 0; i < points.size(); ++i){
        cv::Vec3f &point_on_model = front_intersection_image_.at<cv::Vec3f>(points[i].y, points[i].x);
        tracked_points_.push_back(TrackedPoint(point_on_model, points[i]));
      }

      previous_frame_gray_ = gray;

    }

    cv::Mat CreateMask(){
      cv::Mat mask = cv::Mat::zeros(front_intersection_image_.size(), CV_8UC1);
      for (int r = 0; r < mask.rows; ++r){
        for (int c = 0; c < mask.cols; ++c){
          const cv::Vec3f &f = front_intersection_image_.at<cv::Vec3f>(r, c);
          if (f[0] == GL_FAR && f[1] == GL_FAR && f[2] == GL_FAR){
            continue;
          }
          mask.at<unsigned char>(r, c) = 255;
        }
      }
      return mask;
    }

    std::vector<cv::Point2f> GetPointsOnPreviousImage(){
      std::vector<cv::Point2f> pts;
      for (auto pt : tracked_points_){
        pts.push_back(pt.frame_point);
      }
      return pts;
    }

    std::vector<TrackedPoint> tracked_points_;

    cv::Mat previous_frame_gray_;
    cv::Mat current_frame_gray_;
    
    cv::Mat front_intersection_image_; //for masking

    cv::Size win_size_;
    cv::TermCriteria term_crit_;

    boost::shared_ptr<MonocularCamera> camera_;
    
  };



}

#endif