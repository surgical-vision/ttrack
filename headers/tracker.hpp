#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "headers.hpp"

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
    void operator()(cv::Mat *image);

  
    /**
     * Get a ptr to the frame now that the detector has finished classifying it
     * @return cv::Mat * The frame.
     */
    cv::Mat *GetPtrToFinishedFrame();

    /**
     * Toggle tracking on or not. If it's off, init is called on each new frame .
     * @param toggle Set True for on, False for off.
     */
    void Tracking(const bool toggle);

  protected:
    
    /**
     * Initialise the tracker to get a first estimate of the position. At the moment use the MOI tensor method.
     */
    void Init();
    


    bool tracking_; /**< The variable for toggling tracking on or off.  */
    cv::Mat *frame_; /**< The frame that the tracker is currently working on. */

  };

  


}

#endif
