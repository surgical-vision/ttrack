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
#include <boost/thread/mutex.hpp>

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
    
    //typedef std::pair<boost::shared_ptr<sv::Frame>, std::vector<boost::shared_ptr<Model>> > ImageRenderSetContainer;
    //typedef boost::shared_ptr<ImageRenderSetContainer> ImageRenderSet;

    static void Destroy();
    ~TTrack();
    
    /**
     * A factory method for getting a reference to the singleton TTrack class.
     */
    //static boost::shared_ptr<TTrack> Instance(); 
    static TTrack &Instance();

    void operator()() { RunThreaded(); }

    void TTrack::GetUpdate(std::vector<boost::shared_ptr<Model> > &models);

    /**
     * A method to start running the main method of the class. Loops getting a new frame from the video file, classifying it and then detecting the 
     * instruments in it. Per-frame pose is saved in out_posefile file and video in the out_videofile file.
     * @param[in] video_url The url of the video file. Give this relative to root directory.
     */
    //void RunVideo(const std::string &video_url);
    //void RunVideo(const std::string &left_video_url,const std::string &right_video_url);
    
    /**
     * A method to start running the main method of the class. Same features as RunVideo but it inputs still frames from a directory
     * @param[in] image_url The url of the directory where the images are stored. Give this relative to root directory.
     */
    //void RunImages(const std::string &image_url);
    
    /**
     * A method for saving the current frame in output directory. 
     */
    void SaveFrame();
    
    void GetWindowSize(int &width, int &height);

    /**
     * Setup the tracking system with the files it needs to find, localize and track the objects.
     * @param model_parameter_file A path to a file containing the model specifications. Can contain multiple model classes.
     * @param camera_calibration_file A path to a file containing the camera calibration parameters in OpenCV XML format.
     * @param classifier_path A path to a file containing the classifier to be loaded.
     * @param results_dir A directory in which results can be saved.
     * @param classifier_type Specify the type of classifier.
     * @param camera_type Stereo or monocular camera?
     */
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const CameraType camera_type, const std::string &left_media_file,const std::string &right_media_file);
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const CameraType camera_type, const std::string &media_file);
    //void SetUp(std::string root_dir, const ClassifierType classifier_type, const CameraType camera_type);


    //bool GetLatestUpdate(ImageRenderSet &irs); 
    boost::shared_ptr<const sv::Frame> GetPtrToCurrentFrame() const;

  protected:
    
    void SetUp(const std::string &model_parameter_file, const std::string &camera_calibration_file, const std::string &classifier_path, const std::string &results_dir, const ClassifierType classifier_type, const CameraType camera_type);

    //void AddToQueue();

    /**
     * Draw the model at the current pose onto the canvas image ready for it to be saved
     */
    //void DrawModel(cv::Mat &image) const;

    /**
     * Grab a ptr to a new frame. This is the interface to use if reading from images or reading from a videofile.
     * @return The ptr to the frame.
     */
    boost::shared_ptr<sv::Frame> GetPtrToNewFrame();

    /**
     * Get a pointer to the classifier frame from the detection system.
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

    boost::mutex mutex_;
    //std::queue< ImageRenderSet > processed_frames_;

  private:

    TTrack();
    TTrack(const TTrack &);
    TTrack &operator=(const TTrack &);
    static bool constructed_;
    static boost::scoped_ptr<TTrack> instance_;

  };


}

#endif //_TTRACK_H_
