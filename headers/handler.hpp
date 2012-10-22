#ifndef _HANDLER_HPP_
#define _HANDLER_HPP_

#include "headers.hpp"
#include <vector>
#include <utility>
#include <string>

namespace ttrk{

  class Handler{

  public:

    Handler(const std::string &input_url, const std::string &output_url);
    virtual ~Handler();

    typedef std::pair<cv::Mat *,std::string> ImAndName;

    virtual cv::Mat *GetPtrToNewFrame() = 0;
    virtual void SavePtrToFrame(const cv::Mat *image) = 0;
    virtual void SetInputFileName(const std::string &url) = 0;
    virtual void SetOutputFileName(const std::string &url) = 0;   
    void SaveDebug(const std::vector< ImAndName > &to_save) const;


  protected:
    
    std::string input_url_;
    std::string output_url_;

  };

  class VideoHandler : public Handler {

  public:

    VideoHandler(const std::string &input_url, const std::string &output_url);
    virtual cv::Mat *GetPtrToNewFrame();
    virtual void SavePtrToFrame(const cv::Mat *image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);   

  private:
    
    std::string in_videofile_;
    std::string out_videofile_;

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
  };


  class ImageHandler : public Handler {

  public:

    ImageHandler(const std::string &input_url, const std::string &output_url);
    virtual cv::Mat *GetPtrToNewFrame();
    virtual void SavePtrToFrame(const cv::Mat *image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);

  private:

    std::string in_images_;
    std::string masks_;
    std::string out_images_;
    std::vector<std::string> paths_;
    std::vector<std::string>::const_iterator open_iter_;
    std::vector<std::string>::const_iterator save_iter_;

  };


}






#endif
