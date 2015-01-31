#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <vector>

#include "../headers.hpp"
#include "../../deps/image/image/image.hpp"
#include "localizer.hpp"
#include "model/model.hpp"
#include "temporal/temporal.hpp"

namespace ttrk{

  /**
  * @enum LocalizerType
  * The type of frame-by-frame pose localizer to use in tracking.
  */
  enum LocalizerType { PWP3DLocalizer, ArticulatedLevelSetLocalizer, ComponentLS };

 /**
 * @class Tracker
 * @brief The tracking system.
 * An abstract class for tracking objects in video files using a level set based framework. Expects to be given classified images indictating pixel class membership probabilities for the target class.
 */
  
  class Tracker{

  public:

    /**
    * Constructor for trackers.
    * @param[in] 
    */
    Tracker(const std::string &model_parameter_file, const std::string &results_dir);

    /**
    * Destructor
    */
    virtual ~Tracker();

    /**
     * Overload for boost thread call. This function wraps the calls to the model fitting methods
     * @param image The image pulled from the video file or image file.
     * @param found The success of the detection system. Stops tracking and sets the tracker up to perform tracking recovery when object is detected again.
     */
    void operator()(boost::shared_ptr<sv::Frame> image, const bool found);

    /**
    * Run tracking on a frame.
    * @param image The image pulled from the video file or image file.
    * @param found The success of the detection system. Stops tracking and sets the tracker up to perform tracking recovery when object is detected again.
    */
    void Run(boost::shared_ptr<sv::Frame> image, const bool found);

    void RunStep();
  
    bool HasConverged() const { return localizer_->HasConverged(); }

    /**
     * Get a ptr to the frame now that the detector has finished classifying it
     * @return The frame after use.
     */
    boost::shared_ptr<sv::Frame> GetPtrToFinishedFrame();

    /**
     * Toggle tracking on or not. If it's off, init is called on each new frame .
     * @param toggle Set True for on, False for off.
     */
    void Tracking(const bool toggle);

    void GetTrackedModels(std::vector<boost::shared_ptr<Model> > &models);

    void AddStartPose(const std::vector<float> &pose) { starting_pose_HACK.push_back(pose); }

    std::vector<float> GetStartPose(const size_t idx) { return starting_pose_HACK[idx]; }
    
    static LocalizerType LocalizerTypeFromString(const std::string &str);

  protected:

    /**
    * Initialises the Temporal Model.
    * @return The success of the initilisation.
    */    
    bool InitTemporalModels();

    /**
    * Updates the intenal handle to point at the currently classified frame.
    * @param image The classified frame. Also held by the main TTrack class.
    */
    virtual void SetHandleToFrame(boost::shared_ptr<sv::Frame> image);

    /**
    * Initialise the tracker to get a first estimate of the position. This should be customised with whatever initialisation proceedure is appropriate for the tracker in question.
    * @return Whether the tracker was able to initialise successfully.  
    */
    virtual bool Init() = 0;

    struct TemporalTrackedModel {
      boost::shared_ptr<Model> model; /**< Model to track. */
      boost::shared_ptr<TemporalTracker> temporal_tracker; /**< Kalman filter etc for temporal tracking. */
    };

    std::vector< std::vector<float> > starting_pose_HACK; /**< WILL BE REMOVED ONCE INITIALIZERS ARE SORTED. */

    std::vector<TemporalTrackedModel> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    
    std::vector<TemporalTrackedModel>::iterator current_model_; /**< A reference to the currently tracked model. */
    
    boost::shared_ptr<Localizer> localizer_; /**< The localizer to use for frame-to-frame tracking. */
       
    bool tracking_; /**< The variable for toggling tracking on or off.  */

    boost::shared_ptr<sv::Frame> frame_; /**< The frame that the tracker is currently working on. */
    
    std::string model_parameter_file_; /**< The parameter file for the model type this tracker will track. */

    std::string results_dir_; /**< A directory to save the model results. */   

  };

}

#endif
