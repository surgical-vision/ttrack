#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <image/image.hpp>
#include <boost/scoped_ptr.hpp>
#include "../headers.hpp"
//#include "camera.hpp"
#include "kalman.hpp"
#include "localizer.hpp"
#include "model.hpp"

namespace ttrk{

 /**
 * @class Tracker
 * @brief The tracking system.
 * An abstract class for tracking objects in video files using a level set based framework. Expects to be given classified images indictating pixel class membership probabilities for the target class.
 */

  class Tracker{

  public:

    /**
    * Constructor for the class.
    * @param[in] calibration_filename The url of the camera calibration file. Should be in opencv xml format.
    */
    //explicit Tracker(const std::string &calibration_filename);
    Tracker();

    /**
    * Destructor for the class.
    */
    virtual ~Tracker();

    /**
     * Overload for boost thread call. This function wraps the calls to the model fitting methods
     * @param image The image pulled from the video file or image file.
     */
    void operator()(boost::shared_ptr<sv::Frame> image);

  
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
    void SetHandleToFrame(boost::shared_ptr<sv::Frame> image);

    /**
    * Initialise the tracker to get a first estimate of the position. This should be customised with whatever initialisation proceedure is appropriate for the tracker in question.
    * @return Whether the tracker was able to initialise successfully.  
    */
    virtual bool Init() = 0;
    
    /**
    * Update the target object pose using the Kalman Filter.
    * @param[in] The pose estimate from the level set tracking system.
    */
    void UpdatePose(const cv::Mat &pose_estimate);

    

    //StereoCamera camera_; /**< A camera model for projecting points onto the image plane. */
    std::vector<KalmanTracker> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    std::vector<KalmanTracker>::iterator current_model_; /**< A reference to the currently tracked model. */
    boost::scoped_ptr<Localizer> localizer_;

    
    bool tracking_; /**< The variable for toggling tracking on or off.  */
    //boost::shared_ptr<cv::Mat> frame_; /**< The frame that the tracker is currently working on. */
    boost::shared_ptr<sv::Frame> frame_; /**< The frame that the tracker is currently working on. */
    

  private:

    

  };

  
  


}

#endif
