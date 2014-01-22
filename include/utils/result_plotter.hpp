#ifndef __RESULT_PLOTTER_HPP__
#define __RESULT_PLOTTER_HPP__

#include "../deps/image/image/image.hpp"

namespace ttrk {


  struct State {

    float x_pos;
    float y_pos;
    float z_pos;
    float roll;
    float pitch;
    float yaw;

  };

  class ResultPlotter {

  public:

    State state;
    
    boost::shared_ptr<sv::Frame> image_; 

    void UpdateResults();

  protected:

    std::list<State> tracking_state_;


  };




}



#endif
