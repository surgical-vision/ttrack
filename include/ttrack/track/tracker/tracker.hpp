#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <vector>

#include "../../headers.hpp"
#include "../../utils/image.hpp"
#include "../localizer/localizer.hpp"
#include "../model/model.hpp"
#include "../temporal/temporal.hpp"
#include <ttrack/detect/detect.hpp>

namespace ttrk{

  /**
  * @enum LocalizerType
  * The type of frame-by-frame pose localizer to use in tracking.
  */
  enum LocalizerType { LevelSetForest, PWP3D_SIFT, PWP3D_LK, ComponentLS_SIFT, ComponentLS_LK, ArticulatedComponentLS_GradientDescent, ArticulatedComponentLS_WithSamping, CeresLevelSetSolver, PWP3D, ComponentLS, LK, ArticulatedComponentLS_GradientDescent_F2FLK };


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

    /**
    * Run a single localizer step.
    */
    void RunStep();
  
    /**
    * Has the localizer converged.
    * @return Localization status.
    */
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

    /**
    * Get the current set of tracked models.
    * @param[out] models The models we have currently.
    */
    void GetTrackedModels(std::vector<boost::shared_ptr<Model> > &models);

    /**
    * Add start pose from the config file.
    * @param[in] pose The new pose to add.
    */
    void AddStartPose(const std::vector<float> &pose) { starting_pose_HACK.push_back(pose); }

    /**
    * Get the start pose for one of the models.
    * @param[in] idx The model index.
    * @return The pose.
    */
    std::vector<float> GetStartPose(const size_t idx) { return starting_pose_HACK[idx]; }

    /**
    * Convert a string representation of the localizer type to the LocalizerType type.
    * @param[in] str The string representation of the localizer type.
    * @return The returned LocalizerType represetation.
    */
    static LocalizerType LocalizerTypeFromString(const std::string &str);

    /**
    * Get the localizer progress frame for passing to GUI.
    * @return The frame showing localizer progress.
    */
    cv::Mat GetLocalizerProgressFrame() { return localizer_image_; }

    bool IsFirstRun() { return localizer_->IsFirstRun(); }

    void SetLocalizerIterations(const size_t iterations) { localizer_->SetMaximumIterations(iterations); }

    void SetPointRegistrationWeight(const float weight) { localizer_->SetPointRegistrationWeight(weight); }
    void SetArticulatedPointRegistrationWeight(const float weight) { localizer_->SetArticulatedPointRegistrationWeight(weight); }

    void SetupPointTracker(const bool use_rotations, const bool use_translations, const bool use_articulation, const bool use_global_roll_search_first, const bool use_global_roll_search_last){
      localizer_->SetupPointTracker(use_rotations, use_translations, use_articulation, use_global_roll_search_first, use_global_roll_search_last);
    }

    void SetDetectorType(const ClassifierType &classifier_type, const size_t &number_of_labels) {
      classifier_type_ = classifier_type;
      number_of_labels_ = number_of_labels; 
    }

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

    ClassifierType classifier_type_;
    size_t number_of_labels_;

    std::vector< std::vector<float> > starting_pose_HACK; /**< WILL BE REMOVED ONCE INITIALIZERS ARE SORTED. */

    std::vector<TemporalTrackedModel> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    
    std::vector<TemporalTrackedModel>::iterator current_model_; /**< A reference to the currently tracked model. */
    
    boost::shared_ptr<Localizer> localizer_; /**< The localizer to use for frame-to-frame tracking. */
       
    bool tracking_; /**< The variable for toggling tracking on or off.  */

    boost::shared_ptr<sv::Frame> frame_; /**< The frame that the tracker is currently working on. */
    
    std::string model_parameter_file_; /**< The parameter file for the model type this tracker will track. */

    std::string results_dir_; /**< A directory to save the model results. */   

    int frame_count_; /**< Current frame number. */

    cv::Mat localizer_image_; /**< Image from the localizer for visualizing progress in GUI. */

  };

}

#endif
