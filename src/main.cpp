#include "../headers/ttrack.hpp"
#include <boost/program_options/option.hpp>
#if defined _WIN32 || _WIN64
  #include<direct.h>
#elif defined __linux__
#endif

void ConvertColorspace(const cv::Mat &im, cv::Mat &hue, cv::Mat &sat, cv::Mat &o2, cv::Mat &o3){

  hue = cv::Mat::zeros(im.size(),CV_32FC1);
  sat = cv::Mat::zeros(im.size(),CV_32FC1);
  o2 = cv::Mat::zeros(im.size(),CV_32FC1);
  o3 = cv::Mat::zeros(im.size(),CV_32FC1);

  cv::Mat hsv;
  cv::cvtColor(im,hsv,CV_BGR2HSV);

  for(int r=0;r<im.rows;r++){
    for(int c=0;c<im.cols;c++){

      const unsigned char BLUE = im.at<cv::Vec3b>(r,c)[0];
      const unsigned char GREEN = im.at<cv::Vec3b>(r,c)[1];
      const unsigned char RED = im.at<cv::Vec3b>(r,c)[2];

      hue.at<float>(r,c) = hsv.at<cv::Vec3b>(r,c)[0];
      sat.at<float>(r,c) = hsv.at<cv::Vec3b>(r,c)[1];
      o2.at<float>(r,c) = 0.5 * ((float)RED - (float)GREEN);
      o3.at<float>(r,c) = (0.5*(float)BLUE) - 0.25*((float)RED+(float)GREEN);

    }
  }


}

int SumPositivePixels(cv::Mat &mask){
  int count = 0;
  for(int r=0;r<mask.rows;r++){
    for(int c=0;c<mask.cols;c++){
      count += mask.at<unsigned char>(r,c) > 127;
    }
  }
  return count;
}

size_t GetImageSize(cv::Mat &image, cv::Mat &mask){
  const int pos_pixels = SumPositivePixels(mask);
  if (  pos_pixels > 100 ){
    return pos_pixels;
  }else{
    return image.rows * image.cols;
  }
}

void Train(){

  _chdir("scripts/");

  std::ifstream files("new_video/files.txt");
  const std::string training_dir = "new_video/training_images/";
  const std::string mask_dir = "new_video/masks/";
  std::vector<std::string> image_files;

  if(!files.is_open()) throw(std::runtime_error("Error, could not pen file!\n"));

  while(!files.eof()){
    std::string f;
    files >> f;
    if (f == "") continue;
    image_files.push_back(f);
  }

  size_t data_size = 0;
  for(auto i=image_files.begin();i != image_files.end(); i++){
    cv::Mat im = cv::imread(training_dir + *i);
    cv::Mat mask = cv::imread(mask_dir + *i,0);
    data_size += GetImageSize(im,mask);
  }

  cv::Mat training_data = cv::Mat::zeros(cv::Size(4,data_size),CV_32FC1);
  cv::Mat labels = cv::Mat::zeros(cv::Size(1,data_size),CV_32SC1);

  size_t data_row = 0;
  for(auto i=image_files.begin();i != image_files.end(); i++){
    cv::Mat im = cv::imread(training_dir + *i);
    cv::Mat mask = cv::imread(mask_dir + *i,0);

    bool positive_image = SumPositivePixels(mask) > 100;

    cv::Mat hue,sat,o2,o3;
    ConvertColorspace(im,hue,sat,o2,o3);
    for(int r=0; r<im.rows;r++){
      for(int c=0; c<im.cols;c++){

        bool val = mask.at<unsigned char>(r,c) > 127;
        if( !val && positive_image ) continue;

        training_data.at<float>(data_row,0) = hue.at<float>(r,c);
        training_data.at<float>(data_row,1) = sat.at<float>(r,c);
        training_data.at<float>(data_row,2) = o2.at<float>(r,c);
        training_data.at<float>(data_row,3) = o3.at<float>(r,c);
        
        labels.at<int32_t>(data_row,0) = (int32_t)val;
        data_row++;

      }
    }
  }

  if(data_row != data_size) throw(std::runtime_error("Error, thing"));

  const float priors[2] = {1.0,1.0};

  CvRTParams params(10, //max depth of trees
    500, //minimum sample count at each leaf for a split
    0.0, //minimum regression accuracy (ignored)
    false, //use surrogates
    10, //maximum number of categories to cluster - ignored in 2 class case
    priors, //priors
    true, //calculate the variable importance
    0, //size of random subsets (0 = sqrt(N))
    50, //max number of trees
    0.01, //accuracy
    CV_TERMCRIT_ITER | CV_TERMCRIT_EPS); //halting criteria


  CvMat *var_type = cvCreateMat(training_data.cols+1,1,CV_8U);
  cvSet(var_type,cvScalarAll(CV_VAR_ORDERED)); //data is ordered (can be compared)
  cvSetReal1D(var_type,training_data.cols,CV_VAR_CATEGORICAL); //labels are categorical
  cv::Mat var_type_(var_type,true);

#ifdef DEBUG
  std::cout << "Training...";
  std::cout.flush();
#endif

  const int num = training_data.rows;
  for(int r=0;r<num;r++){


    int32_t val = labels.at<int32_t>(r,0) ;
    assert(val == 0 || val == 1);


  }

  CvRTrees forest_;

  forest_.train(training_data,
    CV_ROW_SAMPLE, //samples are in row form
    labels,
    cv::Mat(),//variable index, used to mask certain features from the training
    cv::Mat(),//sample index, used to mask certain samples entirely
    var_type,//variable type (regression or classifiaction)
    cv::Mat(),//missing data mask
    params);


#ifdef DEBUG
  std::cout << " Done" << std::endl;
#endif

  
  forest_.save( "forest.xml" );

  cvReleaseMat(&var_type);

  
}

int main(int argc, char **argv){

  ttrk::TTrack &t = ttrk::TTrack::Instance();

  try{
    
#if defined(_WIN32) || defined(_WIN64)
     _chdir("../");
#endif
     
     //Train();
     //return 0;

     //construct the helper classes and train the classifier     
     t.SetUp("./data/new_video/",ttrk::RF,ttrk::STEREO);
    //t.SetUp("./data/in_vivo/",ttrk::RF,ttrk::MONOCULAR);
     
     t.RunVideo("left.avi","right.avi");
     //t.RunVideo("short.avi");
     //t.RunImages("calib/point_calib/");
     
     //t.SetUp("./data/test_video/",ttrk::RF,ttrk::STEREO);
     //t.RunVideo("video.avi");

  }catch(std::runtime_error &e){

    std::cerr << e.what() << "\n";
#if defined(_WIN32) || defined(_WIN64)
    system("pause");
#endif
  
  }
  
  ttrk::TTrack::Destroy();

#if defined(_WIN32) || defined(_WIN64)
  _CrtDumpMemoryLeaks();
  system("pause");
#endif

  return 0;

}



