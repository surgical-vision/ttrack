#include "../headers/headers.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>

void GetImageURLAndSize(const std::string &root_url, std::vector<std::string> &urls,
                   size_t &cols, size_t &rows){

  using namespace boost::filesystem;
  
  path root_dir(root_url);
  if(!is_directory(root_dir)){
    throw std::runtime_error("Error, " + root_url + " is not a valid directory.\n");
  }
  //null directory_iterator constructor returns an "end" iterator
  for(directory_iterator itr(root_dir); itr!=directory_iterator(); itr++){ 
    if(!IS_IMAGE(itr->path().extension())) continue;
    urls.push_back(itr->path().filename());
    cv::Mat tmp = cv::imread(urls.back());
    if(tmp.data == 0){
      std::cerr << "Unable to read image " + urls.back() + "\n";
      urls.pop_back();
      continue;
    }
    cols+=tmp.cols;
    rows+=tmp.rows;   
  }

}
