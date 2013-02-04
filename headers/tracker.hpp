#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>
#include "headers.hpp"
#include "camera.hpp"
#include "model.hpp"

namespace ttrk{

  struct KalmanTracker {

    cv::KalmanFilter filter_;
    boost::shared_ptr<Model> model_;
    cv::Mat temporary_update_pose;
    
  };


/**
 * @class Tracker
 * @brief The tracking system.
 * An abstract class for tracking objects in video files using a level set based framework. Expects to be given classified images indictating pixel class membership probabilities for the target class.
 */

  class Tracker{

  public:

    /**
    * Constructor for the class.
    * @param[in] calibration_filename The url of the calibration file. Should be in opencv xml format.
    */
    explicit Tracker(const std::string &calibration_filename);
    
    /**
    * Destructor for the class.
    */
    ~Tracker();

    /**
     * Overload for boost thread call. This function wraps the calls to the model fitting methods
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

    void GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection);

    void SetHandleToFrame(boost::shared_ptr<cv::Mat> image);

    /**
     * Initialise the tracker to get a first estimate of the position. At the moment use the MOI tensor method.
     */
    virtual bool Init() = 0;
    
    const cv::Mat ProjectShapeToSDF();
    void ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const;
    cv::Mat TrackTargetInFrame();
    void FindROI(const std::vector<cv::Vec2i> &convex_hull);
    void UpdatePose(const cv::Mat &pose_estimate);
    double GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const;
    cv::Mat GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf);
    void ScaleJacobian(cv::Mat &jacobian) const;
    void ApplyGradientDescentStep(const cv::Mat &jacobian);
    cv::Mat GetPoseUpdateEstimate() ;

    StereoCamera camera_;
    std::vector<KalmanTracker> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    std::vector<KalmanTracker>::iterator current_model_;
    
    cv::Mat ROI;
    bool tracking_; /**< The variable for toggling tracking on or off.  */
    boost::shared_ptr<cv::Mat> frame_; /**< The frame that the tracker is currently working on. */

  private:

    Tracker();

  };

  
  


}

#endif
