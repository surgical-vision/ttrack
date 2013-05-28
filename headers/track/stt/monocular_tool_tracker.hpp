#ifndef __MONO_TOOL_TRACKER_HPP__
#define __MONO_TOOL_TRACKER_HPP__
#include "surgical_tool_tracker.hpp"

namespace ttrk {

  class MonocularToolTracker : public SurgicalToolTracker {

  public:
    MonocularToolTracker(const int radius, const int height, const std::string &calibration_filename);
    
    virtual ~MonocularToolTracker() {};

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init();
  
  protected:

    MonocularCamera camera_;
  };


}


#endif
