#ifndef _SURGICAL_TOOL_TRACKER_HPP_
#define _SURGICAL_TOOL_TRACKER_HPP_
#include "../../headers.hpp"
#include "tracker.hpp"
#include "../../utils/camera.hpp"

namespace ttrk{

  /**
  * @class SurgicalToolTracker
  * @brief An implementation of a tracking system to track MIS instruments.
  */

  class SurgicalToolTracker : public Tracker{

  public:
    
    /**
    * Construct a trackable tool specifying the width and height of the tool in metric units and a calibration filename for the cameras.
    * @param[in] model_parameter_file The url of the file containing the model to be tracked.
    * @param[in] results_dir The directory where to save the results from tracked models.
    */
    SurgicalToolTracker(const std::string &model_parameter_file, const std::string &results_dir);

    /**
    * The destructor.
    */
    virtual ~SurgicalToolTracker(){};

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init() = 0;
    
  protected:

    /**
    * Initialize the pose of the tool from a configuation file.
    * @param[in] tm The tracked model to initialize the pose of.
    * @param[in] idx The index of the model
    */
    void InitFromFile(boost::shared_ptr<Model> tm, size_t idx);


  };

}


#endif //_SURGICAL_TOOL_TRACKER_HPP_
