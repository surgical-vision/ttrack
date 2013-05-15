#include "../headers/train.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

using namespace ttrk;

void Train::TestClassifier(BaseClassifier &to_train, boost::shared_ptr<cv::Mat> train, boost::shared_ptr<cv::Mat> truth){
  //for sample in test 
  //compare results to truth

}

void Train::LoadImages(const std::vector<std::string> &image_urls, const std::vector<std::string> &mask_urls, const ImageMaskSet::LoadType type){

  //set the index for the training matrix inserter.
  static size_t index = 0;

  //for each image
  for(size_t im=0;im<image_urls.size();im++){

    //load the image and the mask
    cv::Mat image = cv::imread(image_urls[im]);
    cv::Mat mask;

    //only load a mask if there actually is one (positive data or normal data)
    if(type == ImageMaskSet::POSITIVE || type == ImageMaskSet::BOTH){
      mask = cv::imread(mask_urls[im]);
      mask = ConvertMatSingleToTriple(mask);
    }else{ //type == NEGATIVE
      mask = cv::Mat::zeros(image.size(),CV_8UC3);
    }

    if(image.data == 0x0 || mask.data == 0x0){
      std::cerr << "Error reading " << image_urls[im] << ". Continuing...\n" << std::endl;
      continue;
    }

    try{

      //create the ndimage and load its pixels to the training data mask
      NDImage nd_image_(image);
      LoadPixels(nd_image_,mask,type,index);
      std::cout << "Loaded: " << image_urls[im] << std::endl;

    }catch(std::runtime_error &e){
      std::cerr << "Error, image " << image_urls[im] << " failed to load.\n" << e.what() << "\n";
      SAFE_EXIT(); //cannot continue as training matrix will be messed up
    }catch(std::bad_alloc &e){
      std::cerr << "Error allocating memory for NDImage.\n" << e.what() << "\n";
      SAFE_EXIT();
    }

    //delete nd_image_;
  
  }
  
  std::cerr << index+1 << " == " << training_labels_->rows << "\n";

}

void Train::LoadPixels(const NDImage &nd_image, const cv::Mat &mask, const ImageMaskSet::LoadType type, size_t &index){
  
  assert(nd_image.rows() == mask.rows);
  assert(nd_image.cols() == mask.cols);
  assert(mask.type() == CV_8UC3);

  const int rows = nd_image.rows(); 
  const int cols = nd_image.cols();

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      //if the image-mask pair is for a positive image we want to ignore the background pixels
      if(type == ImageMaskSet::POSITIVE && mask.at<cv::Vec3b>(r,c) == cv::Vec3b(0,0,0)) continue;

      const cv::Mat &pix = nd_image.GetPixelData(r,c);
      for(int i=0;i<pix.cols;i++)
        training_data_->at<float>((int)index,i) = pix.at<float>(0,i);
      
      const cv::Vec3b tmp = mask.at<cv::Vec3b>(r,c);
      training_labels_->at<int32_t>((int)index,0) = (uint32_t)class_index_to_mask_[tmp];
      index++;  
    }
  }



}

Train::Train(boost::shared_ptr<std::string> root_dir, const std::string &class_file):root_dir_(root_dir){

  //read classes from xml
  using boost::property_tree::ptree;
  ptree pt;

  try{

    read_xml(class_file,pt); //read the class config values from the xml file
  
  }catch(boost::exception &){

    std::cerr << class_file << std::endl;

    std::cerr << "Error, no config file found or it is corrupt.\nRun training_setup.py in the scripts file.\n";
    SAFE_EXIT();

  }

  //iterate over the class index - RGB pairs pushing them to the map.
  BOOST_FOREACH(const ptree::value_type &v, pt){
    
    //load the index and assoicated rbg value from the xml file
    const size_t index = v.second.get<size_t>("index",0);
    const uint8_t red = v.second.get<uint8_t>("red",0);
    const uint8_t green = v.second.get<uint8_t>("green",0);
    const uint8_t blue = v.second.get<uint8_t>("blue",0);

#ifdef DEBUG
    std::cout << "Constructed Class: " << index << ", (" << (int)red << "," << (int)green << "," << (int)blue << ")" << std::endl;
#endif

    //push them to the map
    const uint8_t RGB[3] = {red,green,blue};
    std::pair<cv::Vec3b,size_t> new_pair;
    new_pair.first = cv::Vec3b(RGB); new_pair.second = index;
    class_index_to_mask_.insert( new_pair );
    
  }

#ifdef DEBUG
  std::cout << "Added " << class_index_to_mask_.size() << " classes.\n";
#endif

}

Train::Train(){}

Train::~Train(){
   
}

