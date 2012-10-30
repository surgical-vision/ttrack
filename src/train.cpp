#include "../headers/train.hpp"
#include "../headers/helpers.hpp"
#include <boost/filesystem.hpp>

using namespace ttrk;

void TrainData::LoadImages(const std::vector<std::string> &image_urls, const std::vector<std::string> &mask_urls, const LoadType type){

  //for each image
  for(size_t im=0;im<image_urls.size();im++){

    //load the image and the mask
    cv::Mat image = cv::imread(image_urls[im]);
    cv::Mat mask;

    if(type == POSITIVE || type == BOTH)
      mask = cv::imread(mask_urls[im]);
    else //type == NEGATIVE
      mask = cv::Mat::zeros(image.size(),image.type());

	//create the ndimage and load its pixels to the training data mask
    NDImage *nd_image_;
    try{

      nd_image_ = new NDImage(image);
      LoadPixels(nd_image_,mask,type);

    }catch(std::runtime_error &e){
      std::cerr << "Error, image " << image_urls[im] << " failed to load.\n" << e.what() << "\n";
      exit(1); //cannot continue as training matrix will be messed up
    }catch(std::bad_alloc &e){
      std::cerr << "Error allocating memory for NDImage.\n" << e.what() << "\n";
      exit(1);
    }

    delete nd_image_;
  
  }

}

void TrainData::LoadCrossValidationData(){

  const std::string im_dir(*root_dir_ + "/data/images/");
  const std::string mask_dir(*root_dir_ + "/data/masks/");

  if(!boost::filesystem::exists(boost::filesystem::path(im_dir)) ||
     !boost::filesystem::exists(boost::filesystem::path(mask_dir)) )
    throw(std::runtime_error("Error, incorrect directory structure. To use cross validation create data/images and data/masks directories in the root directory of the dataset."));

  
  //vectors to store the urls
  std::vector<std::string> ims;
  std::vector<std::string> masks;
  int num_pix=0;
  
  //get the file urls 
  try{
    GetImageURL(im_dir,ims);
    GetImageURL(mask_dir,masks);
  }catch(std::runtime_error &e){
    cerr << "Error, failed to load image urls from directory.\n" << e.what() << "\nExiting...\n";
    exit(1);
  }
  
  //get the training sizes, true means just count the positive pixels false means count whole image
  GetTrainingSize(masks,num_pix,true);
  
  //preallocate training data storage
  training_data_ = new cv::Mat(cv::Size(NDImage::channels_,num_pix),CV_32FC1);
  training_labels_ = new cv::Mat(cv::Size(1,num_pix),CV_32SC1);

  //load the foreground/background images
  LoadImages(ims,masks,BOTH);

}


void TrainData::LoadPixels(const NDImage *nd_image_, const cv::Mat &mask, const LoadType type){

  //check data
  if(nd_image_->bad())
    throw std::runtime_error("Error, image data was corrupt/null");
  if(mask.data == 0)
    throw std::runtime_error("Error, mask data was corrupt/null");

  //check size
  const int rows = nd_image_->rows();
  const int cols = nd_image_->cols();
  if(mask.rows != rows || mask.cols != cols)
    throw std::runtime_error("Error, image and mask have different dimensions");

  const unsigned char *mask_ptr = reinterpret_cast<const unsigned char *>(mask.data);  
  const int chans = mask.channels();

  //keep current index of training/responses matrix
  static int count=0;
  
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      //copy the labels      
      const int index = (r * cols) + c;
      unsigned char mask_val = mask_ptr[index*chans];
      const cv::Mat &tmp = nd_image_->GetPixelData(r,c);

      //if positive image, just copy the positive pixels to the training data
      if(type == POSITIVE){
        if(mask_val > 127){
          training_labels_->at<size_t>(count,0) = 1;
          for(int i=0;i<training_data_->cols;i++)
            training_data_->at<float>(count,i) = tmp.at<float>(0,i);
          count++;
        }
      }
      
      if(type == NEGATIVE){
        for(int i=0;i<training_data_->cols;i++)
          training_data_->at<float>(count,i) = tmp.at<float>(0,i);
        training_labels_->at<size_t>(count,0) = 0;      
        count++;
      }

      if(type == BOTH){
        for(int i=0;i<training_data_->cols;i++)          
          training_data_->at<float>(count,i) = tmp.at<float>(0,i);
        training_labels_->at<size_t>(count,0) = mask_val > 127;
        count++;
      }
  
      
    }
  }

#if defined(DEBUG) || defined(_DEBUG)
  assert(count <= training_labels_->rows && count <= training_data_->rows);
#endif

}

void TrainData::LoadTrainingData(bool cross_validate){

  // set up directories
  const std::string positive_im_dir(*root_dir_ + "/data/positive_data/training_images/");
  const std::string positive_mask_dir(*root_dir_ + "/data/positive_data/masks/");
  const std::string negative_im_dir(*root_dir_ + "/data/negative_data/");
  
  //vectors to store the urls
  std::vector<std::string> positive_ims;
  std::vector<std::string> negative_ims;
  std::vector<std::string> positive_masks;
  int num_pix=0;
  
  //get the file urls 
  GetImageURL(positive_im_dir,positive_ims);
  GetImageURL(negative_im_dir,negative_ims);
  GetImageURL(positive_mask_dir,positive_masks);
  
  //get the training sizes, true means just count the positive pixels false means count whole image
  GetTrainingSize(positive_masks,num_pix,true);
  GetTrainingSize(negative_ims,num_pix,false);
  
  //preallocate training data storage
  training_data_ = new cv::Mat(cv::Size(NDImage::channels_,num_pix),CV_32FC1);
  training_labels_ = new cv::Mat(cv::Size(1,num_pix),CV_32SC1);

  if(cross_validate){
    //load the foreground/background images
    LoadImages(positive_ims,positive_masks,BOTH);
  }else{
    //load the separate training data
    LoadImages(positive_ims, positive_masks, POSITIVE );
    LoadImages(negative_ims, std::vector<std::string>(), NEGATIVE );
  }

}

TrainData::TrainData(std::string &root_dir):root_dir_(&root_dir),training_data_(0x0),training_labels_(0x0){}

TrainData::TrainData():root_dir_(0x0),training_data_(0x0),training_labels_(0x0){}

TrainData::~TrainData(){
   
  delete training_data_;
  training_data_ = 0x0;
   
  delete training_labels_;
  training_labels_ = 0x0;

}
