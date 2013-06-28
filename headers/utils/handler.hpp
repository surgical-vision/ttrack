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

    typedef std::pair<boost::shared_ptr<cv::Mat>,std::string> ImAndName;

    virtual boost::shared_ptr<cv::Mat> GetPtrToNewFrame() = 0;
    virtual void SavePtrToFrame(const boost::shared_ptr<cv::Mat> image) = 0;
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
    virtual boost::shared_ptr<cv::Mat> GetPtrToNewFrame();
    virtual void SavePtrToFrame(const boost::shared_ptr<cv::Mat> image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);   

  private:

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
  };


  class ImageHandler : public Handler {

  public:

    ImageHandler(const std::string &input_url, const std::string &output_url);
    virtual boost::shared_ptr<cv::Mat> GetPtrToNewFrame();
    virtual void SavePtrToFrame(const boost::shared_ptr<cv::Mat> image);
    virtual void SetInputFileName(const std::string &url);
    virtual void SetOutputFileName(const std::string &url);

  private:

    std::vector<std::string> paths_;
    std::vector<std::string>::const_iterator open_iter_;
    std::vector<std::string>::const_iterator save_iter_;

  };


}






#endif
