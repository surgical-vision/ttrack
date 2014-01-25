#ifndef _HANDLER_HPP_
#define _HANDLER_HPP_

#include "../headers.hpp"
#include <vector>
#include <utility>
#include <string>
#include "../../deps/image/image/image.hpp"

namespace ttrk{

  class Handler{

  public:

    Handler(const std::string &input_url, const std::string &output_url);
    virtual ~Handler();

    typedef std::pair<cv::Mat,std::string> ImAndName;

    virtual cv::Mat GetNewFrame() = 0;
    virtual void SaveFrame(const cv::Mat image) = 0;
    virtual void SetInputFileName(const std::string &url) = 0;
    virtual void SetOutputFileName(const std::string &url) = 0;   
    void SaveDebug(const std::vector< ImAndName > &to_save) const;
    virtual bool Done() { return done_; }
    virtual int GetFrameWidth() = 0;
    virtual int GetFrameHeight() = 0;

  protected:

    bool done_;
    std::string input_url_;
    std::string output_url_;

  };

  class VideoHandler : public Handler {

  public:

    VideoHandler(const std::string &input_url, const std::string &output_url);
    virtual cv::Mat GetNewFrame();
    virtual void SaveFrame(const cv::Mat image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);   
    virtual int GetFrameWidth() { return (int)cap_.get(CV_CAP_PROP_FRAME_WIDTH); }
    virtual int GetFrameHeight() { return (int)cap_.get(CV_CAP_PROP_FRAME_HEIGHT); }

  private:

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
  };

  class StereoVideoHandler : public VideoHandler {

  public:

    StereoVideoHandler(const std::string &left_input_url,const std::string &right_input_url,const std::string &output_url);
    virtual cv::Mat GetNewFrame();

  private:

    cv::VideoCapture right_cap_;
    cv::VideoWriter right_writer_;

  };

  class ImageHandler : public Handler {

  public:

    ImageHandler(const std::string &input_url, const std::string &output_url);
    virtual cv::Mat GetNewFrame();
    virtual void SaveFrame(const cv::Mat image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);
    virtual int GetFrameWidth() { if(!paths_.size()) return 0; auto i = cv::imread(paths_[0]); return i.cols; }
    virtual int GetFrameHeight() { if(!paths_.size()) return 0; auto i = cv::imread(paths_[0]); return i.rows; }

  private:

    std::vector<std::string> paths_;
    std::vector<std::string>::const_iterator open_iter_;
    std::vector<std::string>::const_iterator save_iter_;

  };



}






#endif
