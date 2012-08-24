#ifndef _TTRACK_H_
#define _TTRACK_H_

/**
 * @mainpage Surgical Instrument Tracking
 * 
 * Information about the project...
 */

#include "headers.hpp"
#include "classifier.hpp"
#include "tracker.hpp"
#include "detect.hpp"
#include <boost/thread.hpp>

/**
 * @namespace ttrk
 * This is the namespace for the tool tracking project.
 */

namespace ttrk{

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
    void CleanUp();
    ~TTrack();
    
    /**
     * A factory method for getting a reference to the singleton TTrack class.
     */
    static TTrack &Instance(); 

    /**
     * A method to start running the main method of the class. Loops getting a new frame from the video file, classifying it and then detecting the 
     * instruments in it. Per-frame pose is saved in out_posefile file and video in the out_videofile file.
     * @param[in] in_videofile The file where the video is stored.
     * @param[in] out_videofile The file where the output video is saved.
     * @param[in] out_posefile The file where the instrument poses are saved
     */
    void RunVideo(const std::string in_videofile="./data/video.avi", 
                  const std::string out_videofile="./data/results/video.avi",
                  const std::string out_posefile="./data/results/pose.csv");
    
    /**
     * A method to start running the main method of the class. Same features as RunVideo but it inputs still frames from a directory
     * @param[in] in_images The directory where the files are stored.
     * @param[in] out_images The directory where the output files are saved.
     * @param[in] out_posefile The file where the instrument poses are saved
     */
    void RunImages(const std::string in_images="./data/", 
                   const std::string out_images="./data/results/",
                   const std::string out_posefile="./results/pose.csv");

    
    /**
     * The main method of the class. Loops calling the methods of the Tracker and the Detector.
     *
     */
    void Run();

    /**
     * A method for saving the current frame in output directory. 
     */
    void SaveFrame();

    /**
     * Draw the model at the current pose onto the classified image ready for it to be saved
     */
    void DrawModel();

    /** 
     * Save the current images from both the detector and the tracker in the debug directory.
     */
    void SaveDebug() const;

    /**
     * Set verbose mode on or off. This will save lots of output images in the debug directory.
     * @param[in] toggle set either on (true) or off (false).
     */
    void ToggleVerbose(const bool toggle);

    /**
     * Set the filename for reading from an image or video file. The videofile should be set as url + video.avi or if using images they should be named url + image%d.[png|jpg].
     * @param[in] url The full path to the videofile.
     */
    void SetInputFile(const std::string &url);

    /**
     * Set the directory path for writing to a videofile or a set of images. Videos will be saved as url + video.avi whereas images will be saved as url + image%d.png.
     * @param[in] url The full path to write to.
     */
    void SetOutputFile(const std::string &url);


    /**
     * Grab a shared_ptr to a new frame. This is the interface to use if reading from images or reading from a videofile.
     * @return The shared_ptr to the frame.
     */
    boost::shared_ptr<cv::Mat> GetPtrToNewFrame();

  protected:
    
    Tracker tracker_; /**< The class responsible for finding the instrument in the image. */
    Detect detector_; /**< The class responsible for classifying the pixels in the image. */
    
    boost::shared_ptr<cv::Mat> c_frame_; /**< A shared pointer to the current classified frame that will be passed from the classifier to the tracker. */
    boost::shared_ptr<cv::Mat> v_frame_; /**< A newly grabbed video image that is due to be processed by the classifier. */
    
    cv::VideoCapture capture_; /**< A videocapturing class which grabs the frames from videos or a camera. */
    cv::VideoWriter writer_; /**< A videowriting class which saves the frames as they are classified with the projected model drawn on */
    enum InputMode{VIDEO,IMAGE,CAM}; /**< Enum to specify the mode in which the system is running. */
    InputMode mode_; /**< Setting the input mode to video, still image or camera. */
    std::string in_filename_; /**< The filename where the video/images are. If the chosen mode is video then this should be a file url of a video file. If the chosen mode is images then it should be the directory where the images are stored. */

    std::string out_filename_; /**< The filename where the video/images should be saved. If the chosen mode is video then this should be a file url of a video file. If the chosen mode is images then it should be the directory where the images are to be stored. */


  private:

    TTrack();
    TTrack(const TTrack &);
    TTrack &operator=(const TTrack &);
    static bool constructed_;
    static TTrack *instance_;

  };


}

#endif //_TTRACK_H_
