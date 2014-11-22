#ifndef _TTRACK_HPP_
#define _TTRACK_HPP_

/**
 * @mainpage Surgical Instrument Tracking
 * 
 * Information about the project...
 */


#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <queue>

#include "../deps/image/image/image.hpp"
#include "track/tracker.hpp"
#include "detect/detect.hpp"
#include "utils/handler.hpp"

/**
 * @namespace ttrk
 * This is the namespace for the tool tracking project.
 */

namespace ttrk{

   enum CameraType { MONOCULAR = 0, STEREO = 1 };

  /**
   * @class TTrack
   * @brief The interface class for the project. Handles the interaction between all worker classes.
   *
   * This is the interface class of the project. It owns the tracker and the detector as well as
   * performing tasks like saving current frames for validation and the initial interaction with 
   * the filesystem.
   *
   */
  class TTrack{
    
  public:
   
    static void Destroy();
  
    ~TTrack();
    
    /**
     * A factory method for getting a reference to the singleton TTrack class.
     * @return The ttrack instance.
     */
    static TTrack &Instance();

    /**
    * Just check if the ttrack class has been setup fully.
    * @return Whether the instance is running.
    */
    bool IsRunning() { if (handler_ != nullptr) return true; else return false; }

    /**
    * An operator overload to run the tooltracker in a boost thread.
    */
    void operator()() { RunThreaded(); }

    /**
    * Get the currently tracked models for drawing on a GUI for instance.
    * @param[out] models The models to access
    */
    void TTrack::GetUpdate(std::vector<boost::shared_ptr<Model> > &models, const bool force_new_frame);

    /**
     * A method for saving the current frame in output directory. 
     */
    void SaveFrame();
    
    /**
     * Setup the tracking system with the files it needs to find, localize and track the objects for stereo inputs.
     * @param[in] model_parameter_file A path to a file containing the model specifications. Can contain multiple model classes.
     * @param[in] camera_calibration_file A path to a file containing the camera calibration parameters in OpenCV XML format.
     * @param[in] classifier_path A path to a file containing the classifier to be loaded.
     * @param[in] results_dir A directory in which results can be saved.
     * @param[in] classifier_type Specify the type of classifier.
     * @param[in] left_media_file The left input video.
     * @param[in] right_media_file The right input video.
     * @param[in] starting_pose Temporary, will be removed. Just a hack until the initializer is done.
     */
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const std::string &left_media_file, const std::string &right_media_file, const std::vector<float> &starting_pose);
    
    /**
    * Setup the tracking system with the files it needs to find, localize and track the objects for monocular inputs.
    * @param[in] model_parameter_file A path to a file containing the model specifications. Can contain multiple model classes.
    * @param[in] camera_calibration_file A path to a file containing the camera calibration parameters in OpenCV XML format.
    * @param[in] classifier_path A path to a file containing the classifier to be loaded.
    * @param[in] results_dir A directory in which results can be saved.
    * @param[in] classifier_type Specify the type of classifier.
    * @param[in] media_file The input video.
    */
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const std::string &media_file, const std::vector<float> &starting_pose);

    /**
    * Quick method to convert a string formulation of a classifier type (e.g. RF or SVM) to the ClassifierType.
    * @param[in] classifier_name The name of the classifier. SVM for Support Vector Machine, RF for Random Forest and NB for Naive Bayes.
    * @return The classifier type.
    */
    static ClassifierType ClassifierFromString(const std::string &classifier_name);

    /**
    * Quick method to convert a pose in string format to a representation.
    * @param[in] pose_as_string The pose in string form.
    * @return The pose.
    */
    static std::vector<float> PoseFromString(const std::string &pose_as_string);

    /**
    * Get a pointer to the current frame we are processing.
    * @return A pointer to the currently operated on frame.
    */
    boost::shared_ptr<const sv::Frame> GetPtrToCurrentFrame() const;

  protected:
   
    /**
    * Internal method to setup the system after setting up the video input output system.
    * @param[in] model_parameter_file A path to a file containing the model specifications. Can contain multiple model classes.
    * @param[in] camera_calibration_file A path to a file containing the camera calibration parameters in OpenCV XML format.
    * @param[in] classifier_path A path to a file containing the classifier to be loaded.
    * @param[in] results_dir A directory in which results can be saved.
    * @param[in] classifier_type Specify the type of classifier.
    * @param[in] camera_type Whether we are doing monocular or stereo work.
    */
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const CameraType camera_type, const std::vector<float> &starting_pose);

    /**
     * Grab a ptr to a new frame. This is the interface to use if reading from images or reading from a videofile.
     * @return The ptr to the frame.
     */
    boost::shared_ptr<sv::Frame> GetPtrToNewFrame();

    /**
     * Get a pointer to the classifier frame from the detection system.
     * @return The classified frame.
     */
    boost::shared_ptr<sv::Frame> GetPtrToClassifiedFrame();
    
    /**
     * The main method of the class. Called by RunImages or RunVideo which do the
     * appropriate setup calls first. Loops calling the methods of the Tracker and the Detector.
     */
    void RunThreaded();
    
    /**
     * Save the results, if required
     */
    void SaveResults() const;

    boost::scoped_ptr<Tracker> tracker_; /**< The class responsible for finding the instrument in the image. */
    boost::scoped_ptr<Detect> detector_; /**< The class responsible for classifying the pixels in the image. */
    boost::scoped_ptr<Handler> handler_; /**< Pointer to either an ImageHandler or a VideoHandler which handles getting and saving frames with a simple interface */
    boost::shared_ptr<sv::Frame> frame_; /**< A pointer to the current frame that will be passed from the classifier to the tracker. */
    
    std::string results_dir_; /**< A string containing the results directory for the data in use. */
    
    CameraType camera_type_;

  private:

    TTrack();
    TTrack(const TTrack &);
    TTrack &operator=(const TTrack &);
    static bool constructed_;
    static boost::scoped_ptr<TTrack> instance_;

  };


}

#endif //_TTRACK_H_
