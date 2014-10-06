#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>

#include "../headers.hpp"
#include "../../deps/image/image/image.hpp"
#include "kalman.hpp"
#include "localizer.hpp"
#include "model/articulated_model.hpp"


namespace ttrk{

 /**
 * @class Tracker
 * @brief The tracking system.
 * An abstract class for tracking objects in video files using a level set based framework. Expects to be given classified images indictating pixel class membership probabilities for the target class.
 */

  class Tracker{

  public:

    Tracker();
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
     * Get a ptr to the frame now that the detector has finished classifying it
     * @return The frame after use.
     */
    boost::shared_ptr<sv::Frame> GetPtrToFinishedFrame();

    virtual void DrawModelOnFrame(const KalmanTracker &tracked_model, cv::Mat canvas) const = 0;

    /**
     * Toggle tracking on or not. If it's off, init is called on each new frame .
     * @param toggle Set True for on, False for off.
     */
    void Tracking(const bool toggle);

    std::vector<KalmanTracker> TrackedModels() { return tracked_models_; }


  protected:

    /**
    * Initialises the Kalman Filter by setting the state transition matrix, the measurement matrix and the coviariance matrices.
    * @return The success of the initilisation.
    */    
    bool InitKalmanFilter();

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

    std::vector<KalmanTracker> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    std::vector<KalmanTracker>::iterator current_model_; /**< A reference to the currently tracked model. */
    boost::shared_ptr<Localizer> localizer_;

    
    bool tracking_; /**< The variable for toggling tracking on or off.  */
    boost::shared_ptr<sv::Frame> frame_; /**< The frame that the tracker is currently working on. */
    std::string model_parameter_file_;

  private:

    

  };

  
  


}

#endif
