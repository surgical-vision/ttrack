#include "../../include/ttrack/utils/handler.hpp"
#include <boost/filesystem.hpp>
#include <cinder/app/App.h>

using namespace ttrk;

Handler::Handler(const std::string &input_url, const std::string &output_url):
  done_(false),
  input_url_(input_url),
  output_url_(output_url){}

Handler::~Handler(){}

VideoHandler::VideoHandler(const std::string &input_url, const std::string &output_url):
  Handler(input_url,output_url)
  {

  cap_.open(input_url);
  if(!cap_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + input_url_ + "\nPlease enter a new filename.\n");
  }

  
}

VideoHandler::~VideoHandler(){

  if (cap_.isOpened())
   cap_.release();

}

StereoVideoHandler::~StereoVideoHandler(){

  if (!right_cap_.isOpened()){
    right_cap_.release();
  }

}

StereoVideoHandler::StereoVideoHandler(const std::string &left_input_url,const std::string &right_input_url,const std::string &output_url):
  VideoHandler(left_input_url,output_url)
  {

  right_cap_.open(right_input_url);
  if(!right_cap_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + right_input_url + "\nPlease enter a new filename.\n");
  }

}

  
cv::Mat StereoVideoHandler::GetNewFrame(){

  //create one big frame to return the image data in
  cv::Mat right_frame;
  right_cap_ >> right_frame; 
  cv::Mat to_return(right_frame.rows,2*right_frame.cols,right_frame.type());

  //load the left frame
  cv::Mat lhs = to_return(cv::Rect(0,0,right_frame.cols,right_frame.rows));
  VideoHandler::GetNewFrame().copyTo(lhs);

  //load the right frame
  cv::Mat rhs = to_return(cv::Rect(right_frame.cols,0,right_frame.cols,right_frame.rows));
  right_frame.copyTo(rhs);

  cv::imwrite("left.png",lhs);
  cv::imwrite("right.png",rhs);

  if(to_return.data == 0x0) { 
    done_ = true;
    return cv::Mat();
  } else {
    return to_return;
  }

}

ImageHandler::ImageHandler(const std::string &input_url, const std::string &output_url):
  Handler(input_url,output_url){

  using namespace boost::filesystem;
  
  // create a directory object
  path in_dir(input_url_),out_dir(output_url_);
  if(!is_directory(in_dir)){
    throw std::runtime_error("Error, " + input_url_ + " is not a valid directory.\nTracking cannot be performed.");
  }
  
  if(!is_directory(out_dir)) create_directory(out_dir);

  // create a vector to save the filenames in the directory
  std::vector<path> images;
  copy(directory_iterator(in_dir),directory_iterator(),back_inserter(images));
  
  if(images.size() == 0){
    throw std::runtime_error("Error, no image files found in directory: " + input_url_ + "\nPlease enter a new filename.\n");
  }

  //push the actual filenames into the paths_ vector
  for(size_t i=0;i<images.size();i++){
    paths_.push_back( images[i].filename().string() );
  }
  std::cout.flush();

  open_iter_ = save_iter_ = paths_.begin();

}



cv::Mat ImageHandler::GetNewFrame(){

  
  if(open_iter_ == paths_.end()) {
    done_ = true;
    return cv::Mat(); //return an empty shared ptr
  }
  
  //load next image in the list and return it
  cv::Mat to_return = cv::imread(input_url_ + "/" + *open_iter_);
  
  open_iter_++;
  if(to_return.data == 0x0) std::cout << "Error, no data" << std::endl;
  return to_return;

}

cv::Mat VideoHandler::GetNewFrame(){

  cv::Mat to_return;
  cap_ >> to_return;

  if(to_return.data == 0x0) { 
    done_ = true;
    return cv::Mat();
  } else {
    return to_return;
  }

}

void ImageHandler::SaveFrame(const cv::Mat image){

  if(save_iter_ == paths_.end()) throw std::runtime_error("Error, attempt to save image with no file path available.\n");
  
  if(!cv::imwrite(output_url_ + "/" + *save_iter_,image)) 
    throw std::runtime_error("Error, failed to write to path: " + output_url_ + "/" + *save_iter_ );

  save_iter_++;

}

void VideoHandler::SaveFrame(const cv::Mat image){

  if(!writer_.isOpened()){  
  // open the writer to create the processed video
    writer_.open(output_url_,CV_FOURCC('M','J','P','G'), 25, 
                 cv::Size(image.cols,image.rows));
    if(!writer_.isOpened()){
      throw std::runtime_error("Unable to open videofile: " + output_url_ + " for saving.\nPlease enter a new filename.\n");
    }
  }
  
  
  if(!writer_.isOpened()) throw std::runtime_error("Error, attempt to save frame without available video writer.\n");
  
  cv::Mat rgb(image.size(), CV_8UC3);
  for (int r = 0; r < rgb.rows; ++r){
    for (int c = 0; c < rgb.cols; ++c){
      const auto &t = image.at<cv::Vec4b>(r, c);
      rgb.at<cv::Vec3b>(r, c) = cv::Vec3b(t[0], t[1], t[2]);
    }
  }

  writer_ << rgb;

}

void Handler::SaveDebug(const std::vector< ImAndName > &to_save) const {

  //iterate through list
  //save in ./debug or something

}

void VideoHandler::SetInputFileName(const std::string &url){

}

void VideoHandler::SetOutputFileName(const std::string &url){

}

void ImageHandler::SetInputFileName(const std::string &url){

}

void ImageHandler::SetOutputFileName(const std::string &url){

}
