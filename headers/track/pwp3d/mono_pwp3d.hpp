#ifndef __MONO_PWP3D_HPP__
#define __MONO_PWD3D_HPP__

#include "pwp3d.hpp"
#include "../localizer.hpp"

namespace ttrk {

  class MonoPWP3D : public PWP3D {

  public: 
    
    virtual Pose TrackTargetInFrame(KalmanTracker model, boost::shared_ptr<sv::Frame> frame);

  protected:

    virtual cv::Mat &ROI() { return ROI_; }

    virtual void FindROI(const std::vector<cv::Vec2i> &convex_hull);

  protected:

    cv::Mat ROI_; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */

  };


}


#endif
