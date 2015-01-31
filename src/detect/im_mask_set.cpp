#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "../../include/ttrack/detect/im_mask_set.hpp"
#include "../../include/ttrack/headers.hpp"
#include "../../include/ttrack/utils/helpers.hpp"
#include "../../include/ttrack/utils/exceptions.hpp"

using namespace ttrk;

ImageMaskSet::ImageMaskSet(LoadType type,const std::string &root_dir):type_(type),root_dir_(root_dir){
  
  std::string type_string;

  if(type_ == POSITIVE)
    type_string = "/positive_data/";
  else if(type == NEGATIVE)
    type_string = "/negative_data/";
  else if(type == BOTH)
    type_string = "/normal_data/";
  else
    throw std::runtime_error("Unhandled training data type used. Expects POSITIVE, NEGATIVE or BOTH.\n");

  try{
    
    GetImageURL(root_dir_ + type_string + "training_images/", image_urls_);
    if(type != NEGATIVE)
      GetImageURL(root_dir_ + type_string + "masks/", mask_urls_);

  }catch(FileSysErr &e){

    /*if(type == NEGATIVE){
      if(e.type == FileSysErr::NoFiles){
        std::cerr << "Negative image directory exists but is empty, continue? [y/n]\n";
        char cont; std::cin >> cont;
        if(cont == 'n'){        
          std::cerr << "Exiting...\n"; 
          SAFE_EXIT(); 
        }
      }else{
        std::cerr << "Error reading images from " + e.name + ": ";
        if(e.type == FileSysErr::NoFiles) std::cerr <<  "Directory doesn't contain any png or jpg files.\n";
        else if(e.type == FileSysErr::NoDir) std::cerr <<  "Directory does not exist.\n";
        SAFE_EXIT();
      }
    }*/
#ifdef DEBUG
    if(e.type == FileSysErr::NoFiles)
      std::cerr << "Directory " + root_dir_ + type_string + " is empty.\n";
    else if(e.type == FileSysErr::NoDir)
      std::cerr << "Directory " + root_dir_ + type_string + " does not exist.\n";
#endif
  }
   
  num_pix_ = 0;
  if(type == POSITIVE)
    num_pix_ += GetTrainingSize(image_urls_,true);
  else
    num_pix_ += GetTrainingSize(mask_urls_,false);
  
}


void ImageMaskSet::GetImageURL(const std::string &root_url, std::vector<std::string> &urls){

  using namespace boost::filesystem;
  
  path root_dir(root_url);
  if(!is_directory(root_dir)){
    throw ttrk::FileSysErr(root_url,ttrk::FileSysErr::NoDir);
  }
  //null directory_iterator constructor returns an "end" iterator
  for(directory_iterator itr(root_dir); itr!=directory_iterator(); itr++){ 
    //is the path for an image?
	  if(!IS_IMAGE(itr->path().extension().string())) continue;
    //add the full file path to the vector
    urls.push_back(itr->path().relative_path().generic_string());
  }

  if(urls.size() == 0) 

    throw ttrk::FileSysErr(root_url,ttrk::FileSysErr::NoFiles);

}


int ImageMaskSet::GetTrainingSize(const std::vector<std::string> &urls, const bool count_only_positive) const{
  
  int pix_to_add = 0; //a counter to add to the total number of pixels

  std::vector<std::string>::const_iterator url_it;

  for(url_it = urls.begin();url_it!=urls.end();url_it++){

    cv::Mat tmp = cv::imread(*url_it);

    // if the image is a positive image, just count the number of positives in the mask
    if(count_only_positive){
      int sum = CountNonZero(tmp);
      pix_to_add+=sum;
    }else{ //else just count the pixels as negatives are all of the image
      cv::Mat tmp = cv::imread(*url_it);
      pix_to_add += (tmp.rows * tmp.cols);
    }

  }

  return pix_to_add;
}



int ImageMaskSet::CountNonZero(cv::Mat &im) const{

  unsigned char *n = im.data;
  const int rows = im.rows;
  const int chans = im.channels();
  const int cols = im.cols;
  int ret = 0;
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      for(int chan=0;chan<chans;chan++){
        if(n[((r*cols + c)*chans)+chan] > (unsigned char)127){
          ret++;
          break;
        }
      }
    }
  }
  return ret;  
}

  
