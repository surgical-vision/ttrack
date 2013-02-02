#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>
#include "headers.hpp"
#include "camera.hpp"
#include "model.hpp"

namespace ttrk{

/**
 * @class Tracker
 * @brief The tracking system.
 *
 */

  class Tracker{

  public:

    Tracker();
    ~Tracker();

    /**
     * Overload for boost thread call. This functio wraps the calls to the model fitting methods
     * @param image The image pulled from the video file or image file.
     */
    void operator()(boost::shared_ptr<cv::Mat> image);

  
    /**
     * Get a ptr to the frame now that the detector has finished classifying it
     * @return The frame after use.
     */
    boost::shared_ptr<cv::Mat> GetPtrToFinishedFrame();

    /**
     * Toggle tracking on or not. If it's off, init is called on each new frame .
     * @param toggle Set True for on, False for off.
     */
    void Tracking(const bool toggle);

  protected:
    
    bool InitKalmanFilter();

    void SetHandleToFrame(boost::shared_ptr<cv::Mat> image);

    /**
     * Initialise the tracker to get a first estimate of the position. At the moment use the MOI tensor method.
     */
    virtual bool Init() = 0;
    
    const cv::Mat ProjectShapeToSDF(const int image_width, const int image_height) const;

    const cv::Mat PerformUpdateStep(const cv::Mat &Prediction);
    


    boost::scoped_ptr<StereoCamera> camera_;
    std::vector<Model<cv::Vec3f> *> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    std::vector<Model<cv::Vec3f> *>::iterator current_model_;
    
    bool tracking_; /**< The variable for toggling tracking on or off.  */
    boost::shared_ptr<cv::Mat> frame_; /**< The frame that the tracker is currently working on. */
  
  };

  


}

#endif
