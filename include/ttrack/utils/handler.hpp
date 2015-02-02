#ifndef _HANDLER_HPP_
#define _HANDLER_HPP_

#include <vector>
#include <utility>
#include <string>

#include "../headers.hpp"
#include "image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

namespace ttrk{

  /**
  * @class Handler
  * @brief An abstract interface to video and image loaders.
  * Allows stereo video, monocular video or sequences of images to be used as the input.
  */

  class Handler{

  public:

    /**
    * Construct a video or image capture device.
    * @param[in] input_url The url of the input file
    * @param[in] output_url The url of the output file.
    */
    Handler(const std::string &input_url, const std::string &output_url);
    
    /**
    * Default destructor.
    */
    virtual ~Handler();

    /**
    * @typedef ImAndName A named image type.
    */
    typedef std::pair<cv::Mat,std::string> ImAndName;

    /**
    * Load a new frame from the handler.
    * @return the new image frame.
    */
    virtual cv::Mat GetNewFrame() = 0;

    /**
    * Save a frame at the output url.
    * @param[in] image The output image.
    */
    virtual void SaveFrame(const cv::Mat image) = 0;

    /**
    * Check if there are more files left to load.
    * @return The return status, true for no files left false otherwise.
    */
    virtual bool Done() { return done_; }

    /**
    * Get the width of the frames we are using. Assumes all frames are the same size.
    * @return The frame width.
    */
    virtual int GetFrameWidth() = 0;
    
    /**
    * Get the height of the frames we are using. Assumes all frames are the same size.
    * @return The frame height.
    */
    virtual int GetFrameHeight() = 0;

  protected:

    bool done_; /**< Done status flag. */
    std::string input_url_; /**< URL of files we are loading. */
    std::string output_url_; /**< URL of files we are saving to. */

  };

  class VideoHandler : public Handler {

  public:

    /**
    * Create a monocular video interface.
    * @param[in] input_url The input video file.
    * @param[in] output_url The output video file.
    */
    VideoHandler(const std::string &input_url, const std::string &output_url);

    /**
    * Destructor for the video handler.
    */
    ~VideoHandler();

    /**
    * Get a new frame from the video file. 
    * @return The new frame. 
    */
    virtual cv::Mat GetNewFrame();

    /**
    * Save the current frame to the output video file.
    * @param[in] image The frame to save.
    */
    virtual void SaveFrame(const cv::Mat image);

    /**
    * Get the width of the frames we are using. Assumes all frames are the same size.
    * @return The frame width.
    */
    virtual int GetFrameWidth() { return (int)cap_.get(CV_CAP_PROP_FRAME_WIDTH); }

    /**
    * Get the height of the frames we are using. Assumes all frames are the same size.
    * @return The frame height.
    */
    virtual int GetFrameHeight() { return (int)cap_.get(CV_CAP_PROP_FRAME_HEIGHT); }

  private:

    cv::VideoCapture cap_; /**< The video capture interface. */
    cv::VideoWriter writer_; /**< The video output interface. */
    
  };

  class StereoVideoHandler : public VideoHandler {

  public:

    /**
    * Create a video handler for a stereo video file.
    * @param[in] left_input_url The left eye input video file.
    * @param[in] right_input_url The right eye input video file.
    * @param[in] output_url The output video file.
    */
    StereoVideoHandler(const std::string &left_input_url,const std::string &right_input_url,const std::string &output_url);

    /**
    * Get a new frame from the video file.
    * @return The new frame.
    */
    virtual cv::Mat GetNewFrame();

    /**
    * Destructor, close the file handles.
    */
    ~StereoVideoHandler();

  private:

    cv::VideoCapture right_cap_; /**< The right video capture interface. */
    cv::VideoWriter right_writer_; /**< The right video writer interface. */

  };

  class ImageHandler : public Handler {

  public:

    /**
    * Create a image handler for image sequences.
    * @param[in] input_url The input directory where files are stored.
    * @param[in] output_url The output directory where processed files are saved.
    */
    ImageHandler(const std::string &input_url, const std::string &output_url);
    
    /**
    * Load the next image from the directory.
    * @return The next image.
    */
    virtual cv::Mat GetNewFrame();

    /**
    * Save the current frame in the output directory.
    * @param[in] image The image to save.
    */
    virtual void SaveFrame(const cv::Mat image);

    /**
    * Get the width of the frames we are using. Assumes all frames are the same size.
    * @return The frame width.
    */
    virtual int GetFrameWidth() { if(!paths_.size()) return 0; auto i = cv::imread(paths_[0]); return i.cols; }

    /**
    * Get the width of the frames we are using. Assumes all frames are the same size.
    * @return The frame width.
    */
    virtual int GetFrameHeight() { if(!paths_.size()) return 0; auto i = cv::imread(paths_[0]); return i.rows; }

  private:

    std::vector<std::string> paths_; /**< The image urls to load and process. */
    std::vector<std::string>::const_iterator open_iter_; /**< The images still to open. */
    std::vector<std::string>::const_iterator save_iter_; /**< The images to save. */
     
  };



}






#endif
