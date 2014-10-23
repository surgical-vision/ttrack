#ifndef __TEMPORAL_HPP__
#define __TEMPORAL_HPP__

#include <boost/shared_ptr.hpp>

#include "../model/model.hpp"

namespace ttrk {

  class TemporalTracker {

  public:

    virtual void UpdatePoseWithMotionModel(boost::shared_ptr<Model> model) = 0;

    virtual void Init() = 0;

  protected:

    

  };

  class KalmanFilterTracker : public TemporalTracker {

  public:

    virtual void UpdatePoseWithMotionModel(boost::shared_ptr<Model> model) { };
    
    virtual void Init() {}

  protected:


  };


}


#endif