#ifndef __MONO_PWP3D_HPP__
#define __MONO_PWD3D_HPP__

#include "pwp3d.hpp"
#include "../localizer.hpp"

namespace ttrk {

  class MonoPWP3D : public PWP3D {

  public: 

    MonoPWP3D(boost::shared_ptr<MonocularCamera> camera) : camera_(camera), PWP3D(camera->Width(), camera->Height()) {}

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

    virtual void UpdateErrorAccumulator() {  }

  protected:

    //void GetRenderedModelAtPose(const boost::shared_ptr<Model> model, cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image) const;

    //virtual void GetFastDOFDerivs(const Pose &pose, double *pose_derivs, double *intersection);

    boost::shared_ptr<MonocularCamera> camera_;

  };


}


#endif
