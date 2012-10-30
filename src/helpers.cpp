#include "../headers/headers.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>

void GetImageURL(const std::string &root_url, std::vector<std::string> &urls){

  using namespace boost::filesystem;
  
  path root_dir(root_url);
  if(!is_directory(root_dir)){
    throw std::runtime_error("Error, " + root_url + " is not a valid directory.\n");
  }
  //null directory_iterator constructor returns an "end" iterator
  for(directory_iterator itr(root_dir); itr!=directory_iterator(); itr++){ 
    //is the path for an image?
	  if(!IS_IMAGE(itr->path().extension().string())) continue;
    //add the full file path to the vector
    urls.push_back(itr->path().relative_path().string());
  }

}

void GetTrainingSize(const std::vector<std::string> &urls, int &num_pix, const bool positive){
  
  std::vector<std::string>::const_iterator url_it;
  for(url_it = urls.begin();url_it!=urls.end();url_it++){
    cv::Mat tmp = cv::imread(*url_it);    
    // if the image is a positive image, just count the number of positives in the mask
    if(positive){
      int sum = CountNonZero(tmp);
      num_pix+=sum;
    }else{ //else just count the pixels as negatives are all of the image
      cv::Mat tmp = cv::imread(*url_it);
      num_pix += (tmp.rows * tmp.cols);
    }
  }

}

int CountNonZero(cv::Mat &im){

  unsigned char *n = im.data;
  const int rows = im.rows;
  const int chans = im.channels();
  const int cols = im.cols;
  int ret = 0;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      if(n[(r*cols + c)*chans] > (unsigned char)127)
        ret++;
    }
  }
  return ret;  
}

  
