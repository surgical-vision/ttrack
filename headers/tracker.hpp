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
  * @struct KalmanTracker
  * @brief A container to hold a tracked model and the kalman filter associated with it.
  */
  struct KalmanTracker {

    cv::KalmanFilter filter_; /**< The Kalman Filter used to track the class. */
    boost::shared_ptr<Model> model_; /**< The tracked model. */
    cv::Mat temporary_update_pose; /**< A store for the pose update. May be removed. */
    
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
    * @param[in] calibration_filename The url of the camera calibration file. Should be in opencv xml format.
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

    /**
    * Initialises the Kalman Filter by setting the state transition matrix, the measurement matrix and the coviariance matrices.
    * @return The success of the initilisation.
    */    
    bool InitKalmanFilter();

    /**
    * Finds an intersection between a ray cast from the current pixel through the tracked object.
    * @param[in] r The row index of the pixel.
    * @prarm[in] c The column index of the pixel.
    * @param[out] front_intersection The intersection between the ray and the front of the object.
    * @param[out] back_intersection The intersection between the ray and the back of the object.
    */
    void GetTargetIntersections(const int r, const int c, cv::Vec3f &front_intersection, cv::Vec3f &back_intersection);

    /**
    * Updates the intenal handle to point at the currently classified frame.
    * @param image The classified frame. Also held by the main TTrack class.
    */
    void SetHandleToFrame(boost::shared_ptr<cv::Mat> image);

    /**
    * Initialise the tracker to get a first estimate of the position. This should be customised with whatever initialisation proceedure is appropriate for the tracker in question.
    * @return Whether the tracker was able to initialise successfully.  
    */
    virtual bool Init() = 0;
    
    /**
    * Construct a signed distance function of the outer contour of the shape projected into the image plane.
    * @return The image containin the signed distance function. Will be a single channel floating point image.
    */
    const cv::Mat ProjectShapeToSDF();

    /**
    * Computes the normalization constant of the pose update equation.
    * @param[out] norm_foreground The normalization constant for the foreground class.
    * @param[out] norm_background The normalization constant for the background class.
    * @param[in] sdf_image The signed distance function image.
    */
    void ComputeNormalization(double &norm_foreground, double &norm_background, const cv::Mat &sdf_image) const;
    
    /**
    * Performs the level set based tracking to localise the target object in the frame.
    * @return The updated pose of the tracked object.
    */
    cv::Mat TrackTargetInFrame();

    /**
    * Experimental! Finds and sets a ROI image around the target object. This is to reduce the tracking cost of computing the value of the energy function on pixels which are far from the object contour.
    * @param[in] convex_hull The convex hull of the points which make up the shape.
    */
    void FindROI(const std::vector<cv::Vec2i> &convex_hull);

    /**
    * Update the target object pose using the Kalman Filter.
    * @param[in] The pose estimate from the level set tracking system.
    */
    void UpdatePose(const cv::Mat &pose_estimate);

    /**
    * Compute the first part of the derivative, getting a weight for each contribution based on the region agreements.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] sdf The signed distance function image.
    * @param[in] norm_foreground The normalization constant for the foreground class.
    * @param[in] norm_background The normalization constnat for the background class.
    */
    double GetRegionAgreement(const int r, const int c, const float sdf, const double norm_foreground, const double norm_background) const;
    
    /**
    * Get the second part of the derivative. The derivative of the contour w.r.t the pose parameters.
    * @param[in] r The row index of the current pixel.
    * @param[in] c The column index of the current pixel.
    * @param[in] dSDFdx The derivative of the signed distance function /f$\frac{\partial SDF}{\partial x}/f$ at the current pixel.
    * @param[in] dSDFdy The derivative of the signed distance function /f$\frac{\partial SDF}{\partial y}/f$ at the current pixel.
    * @param[in[ sdf The current value of the signed distance function at the pixel /f$(r,c)/f$.
    * @return The pose derivitives as a vector.
    */
    cv::Mat GetPoseDerivatives(const int r, const int c, const float dSDFdx, const float dSDFdy, const float sdf);
    
    /**
    * Apply some scaling to the pose derivatives to modify the step size.
    * @param[in] jacobian The pose derivatives.
    */
    void ScaleJacobian(cv::Mat &jacobian) const;

    /**
    * Applys one step of gradient descent to the pose. 
    * @param[in] jacobian The pose update of the target object.
    */    
    void ApplyGradientDescentStep(const cv::Mat &jacobian);

    StereoCamera camera_; /**< A camera model for projecting points onto the image plane. */
    std::vector<KalmanTracker> tracked_models_; /**< a vector of tracked models. TODO: switch this out for point cloud mesh or some better data structure. */
    std::vector<KalmanTracker>::iterator current_model_; /**< A reference to the currently tracked model. */
    
    cv::Mat ROI; /**< Experimental feature. Instead of performing the level set tracking over the whole image, try to find a ROI around where the target of interest is located. */
    bool tracking_; /**< The variable for toggling tracking on or off.  */
    boost::shared_ptr<cv::Mat> frame_; /**< The frame that the tracker is currently working on. */

  private:

    Tracker();

  };

  
  


}

#endif
