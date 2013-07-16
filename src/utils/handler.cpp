#include "../../headers/utils/handler.hpp"
#include <boost/filesystem.hpp>

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
  for(size_t i=0;i<images.size();i++)
    paths_.push_back( images[i].filename().string() );

  open_iter_ = save_iter_ = paths_.begin();

}

boost::shared_ptr<cv::Mat> ImageHandler::GetPtrToNewFrame(){

  
  if(open_iter_ == paths_.end()) {
    done_ = true;
    return boost::shared_ptr<cv::Mat>(); //return an empty shared ptr
  }
  
  //load next image in the list and return it
  boost::shared_ptr<cv::Mat> m(new cv::Mat);
  *m = cv::imread(input_url_ + "/" + *open_iter_);

  
  open_iter_++;
  if(m->data == 0x0) std::cout << "Error, no data" << std::endl;
  return m;

}

boost::shared_ptr<cv::Mat> VideoHandler::GetPtrToNewFrame(){

  boost::shared_ptr<cv::Mat> m(new cv::Mat);
  cap_ >> *m;
  
  if(m->data == 0x0) { 
    done_ = true;
    return boost::shared_ptr<cv::Mat>();
  } else {
    return m;
  }

}

void ImageHandler::SavePtrToFrame(boost::shared_ptr<cv::Mat> image){

  if(save_iter_ == paths_.end()) throw std::runtime_error("Error, attempt to save image with no file path available.\n");
  
  if(!cv::imwrite(output_url_ + "/" + *save_iter_,*image)) 
    throw std::runtime_error("Error, failed to write to path: " + output_url_ + "/" + *save_iter_ );

  save_iter_++;

}

void VideoHandler::SavePtrToFrame(const boost::shared_ptr<cv::Mat> image){
  
  if(!writer_.isOpened()){  
  // open the writer to create the processed video
    writer_.open(output_url_,CV_FOURCC('D','I','B',' '), 25, 
                 cv::Size(image->cols,image->rows));
    if(!writer_.isOpened()){
      throw std::runtime_error("Unable to open videofile: " + output_url_ + " for saving.\nPlease enter a new filename.\n");
    }
  }
  
  
  if(!writer_.isOpened()) throw std::runtime_error("Error, attempt to save frame without available video writer.\n");
  writer_ << *image;
  
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
