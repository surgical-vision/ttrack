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

    void operator()(boost::shared_ptr<const cv::Mat> image);

    /**
     * Initialise the tracker to get a first estimate of the position. At the moment use the MOI tensor method.
     * @param image The classified image from the detection class.
     */

    void Init(boost::shared_ptr<const cv::Mat> image);

    

  };




}

#endif
