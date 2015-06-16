#ifndef __MONO_TOOL_TRACKER_HPP__
#define __MONO_TOOL_TRACKER_HPP__

#include "surgical_tool_tracker.hpp"

namespace ttrk {

  /**
  * @class MonocularToolTracker
  * @brief A tool tracked for monocular cameras.
  * This class provides...
  */
  class MonocularToolTracker : public SurgicalToolTracker {

  public:

    /**
    * Construct a trackable tool specifying the width and height of the tool in metric units and a calibration filename for the cameras.
    * @param[in] model_parameter_file The url of the file containing the model to be tracked.
    * @param[in] calibration_filename The url of the file containing the camera calibration.
    * @param[in] results_dir The directory where to save the results from tracked models.
    * @param[in] localizer_type The type of frame-by-frame localizer to use in tracking.
    * @param[in] number_of_labels The number of labels used in detection. This includes the background label.
    */
    MonocularToolTracker(const std::string &model_parameter_file, const std::string &calibration_filename, const std::string &results_dir, const LocalizerType &localizer_type, const size_t number_of_labels);
    
    /**
    * Destructor
    */
    virtual ~MonocularToolTracker() {};

    /**
    * Custom initialisation for the surgical tool. Finds connected regions and then using the moments of these regions, initialises a 
    * an esimate of the 2D pose.
    * @return The success of the initialisation.
    */
    virtual bool Init();
    
  protected:
        
    boost::shared_ptr<MonocularCamera> camera_; /**< The pinhole camera model used to view the scene. */

  };

  
  


}


#endif
