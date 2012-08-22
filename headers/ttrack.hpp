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
    
    ~TTrack();
    
    /**
     * A factory method for getting a reference to the singleton TTrack class.
     */
    static TTrack &Instance(); 

    /**
     * A main method for running the class. Loops getting a new frame from the video file, classifying it and then detecting the 
     * instruments in it. Per-frame pose is saved in out_posefile file and video in the out_videofile file.
     * @param in_videofile The file where the video is stored.
     * @param out_videofile The file where the output video is saved.
     * @param out_posefile The file where the instrument poses are saved
     */
    void RunVideo(const std::string in_videofile="./data/video.avi", const std::string out_videofile="./data/results/video.avi",
                  const std::string out_posefile="./data/results/pose.csv");

    /**
     * A main method for running the class. Same features as RunVideo but it inputs still frames from a directory
     * @param in_images The directory where the files are stored.
     * @param out_images The directorywhere the output files are saved.
     * @param out_posefile The file where the instrument poses are saved
     */
    void RunImages(const std::string in_images="./data/", const std::string out_images="./data/results/",
                   const std::string out_posefile="./results/pose.csv");

     
    /**
     * A method for saving the current frame in debug directory. The filename will be image_X.png 
     * where X is the current frame number
     * @param url The directory for the filename.
     */
    void SaveFrame(const std::string url="./debug/frames/") const;

    /**
     * Set verbose mode on or off. This will save lots of output images in the debug directory.
     * @param toggle set either on (true) or off (false)
     */
    void ToggleVerbose(const bool toggle);

  protected:
    
    Tracker tracker_; /**< The class responsible for finding the instrument in the image. */
    Detect detector_; /**< The class responsible for classifying the pixels in the image. */
    cv::Mat *current_frame_; /**< A pointer to the current classified frame that will be passed from the classifier to the tracker. */

  private:

    TTrack();
    TTrack(const TTrack &);
    TTrack &operator=(const TTrack &);
    static bool constructed_;
    static TTrack *instance_;

  };


}

#endif //_TTRACK_H_
