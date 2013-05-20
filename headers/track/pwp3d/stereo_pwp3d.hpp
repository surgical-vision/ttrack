#ifndef __STEREO_PWP3D_HPP__
#define __STEREO_PWP3D_HPP__

#include "pwp3d.hpp"

namespace ttrk {

  class StereoPWP3D : public PWP3D {
    
  public: 
    virtual cv::Mat TrackTargetInFrame(KalmanTracker &model){
      cv::Mat x;
      return x;

    }

  protected:

    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull);

    boost::shared_ptr<StereoCamera> camera_;
    cv::Mat ROI_left_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    cv::Mat ROI_right_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };




}

#endif
