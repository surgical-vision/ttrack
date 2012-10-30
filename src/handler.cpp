#include "../headers/handler.hpp"
#include <boost/filesystem.hpp>

using namespace ttrk;

Handler::Handler(const std::string &input_url, const std::string &output_url):
  input_url_(input_url),
  output_url_(output_url){}

Handler::~Handler(){}

VideoHandler::VideoHandler(const std::string &input_url, const std::string &output_url):
  Handler(input_url,output_url),
  in_videofile_(input_url + "/" + "video.avi"),
  out_videofile_(output_url + "/output/" + "video.avi")
  {

  cap_.open(in_videofile_);
  if(!cap_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + in_videofile_ + "\nPlease enter a new filename.\n");
  }

  // open the writer to create the processed video
  writer_.open(out_videofile_,CV_FOURCC('M','J','P','G'), 25, 
               cv::Size((int)cap_.get(CV_CAP_PROP_FRAME_HEIGHT),
                        (int)cap_.get(CV_CAP_PROP_FRAME_WIDTH)));
  if(!writer_.isOpened()){
    throw std::runtime_error("Unable to open videofile: " + out_videofile_ + " for saving.\nPlease enter a new filename.\n");
  }

}

ImageHandler::ImageHandler(const std::string &input_url, const std::string &output_url):
  Handler(input_url,output_url),
  in_images_(input_url + "/training_images/"),
  masks_(input_url + "/masks/"),
  out_images_(output_url + "/output/images/"){

  using namespace boost::filesystem;
  
  // create a directory object
  path in_dir(in_images_),out_dir(out_images_);
  if(!is_directory(in_dir)){
    throw std::runtime_error("Error, " + in_images_ + " is not a valid directory.\n");
  }
  
  if(!is_directory(out_dir)) create_directory(out_dir);

  // create a vector to save the filenames in the directory
  std::vector<path> images;
  copy(directory_iterator(in_dir),directory_iterator(),back_inserter(images));
  
  if(images.size() == 0){
    throw std::runtime_error("Error, no image files found in directory: " + in_images_ + "\nPlease enter a new filename.\n");
  }

  //push the actual filenames into the paths_ vector
  for(size_t i=0;i<images.size();i++)
    paths_.push_back( images[i].filename().string() );

  open_iter_ = save_iter_ = paths_.begin();

}

cv::Mat *ImageHandler::GetPtrToNewFrame(){

  if(open_iter_ == paths_.end()) return 0;
  
  //load next image in the list and return it
  cv::Mat *m = new cv::Mat;
  *m = cv::imread(in_images_ + "/" + *open_iter_);

  open_iter_++;
  return m;

}

cv::Mat *VideoHandler::GetPtrToNewFrame(){

  cv::Mat *m = new cv::Mat;
  cap_ >> *m;
  return m;

}

void ImageHandler::SavePtrToFrame(const cv::Mat *image){

  if(save_iter_ == paths_.end()) throw std::runtime_error("Error, attempt to save image with no file path available.\n");
  
  std::cout << *save_iter_ << std::endl;;

  if(!cv::imwrite(out_images_ + "/" + *save_iter_,*image)) 
    throw std::runtime_error("Error, failed to write to path: " + out_images_ + "/" + *save_iter_ );

  save_iter_++;

}

void VideoHandler::SavePtrToFrame(const cv::Mat *image){
  
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
