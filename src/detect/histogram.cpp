#include "../../include/ttrack/detect/histogram.hpp"

using namespace ttrk;


bool Histogram::ClassifyFrame(boost::shared_ptr<sv::Frame> frame){

  cv::Mat whole_frame = frame->GetImage();
  NDImage nd_image(whole_frame);
  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  //size_t pixel_count = 0;

  float *frame_data = (float *)frame->GetClassificationMap().data;

  cv::Mat test_frame_fg(rows, cols, CV_32FC1);
  cv::Mat test_frame_bg(rows, cols, CV_32FC1);
  cv::Mat test_frame_winner(rows, cols, CV_32FC1);

  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      const int index = r*cols + c;

      cv::Mat rgb(3, 1, CV_8UC1);
      rgb.at<unsigned char>(0) = whole_frame.at<cv::Vec3b>(r, c)[0];
      rgb.at<unsigned char>(1) = whole_frame.at<cv::Vec3b>(r, c)[1];
      rgb.at<unsigned char>(2) = whole_frame.at<cv::Vec3b>(r, c)[2];

      cv::Mat pix = nd_image.GetPixelData(r, c);

      float hue = pix.at<float>(0);
      float sat = pix.at<float>(1);
      float o1 = pix.at<float>(2);
      float o2 = pix.at<float>(3);

      test_frame_fg.at<float>(r, c) = PredictProb(pix, 1);
      test_frame_bg.at<float>(r, c) = PredictProb(pix, 0);
      if (test_frame_fg.at<float>(r, c) > test_frame_bg.at<float>(r, c))
        test_frame_winner.at<float>(r, c) = 1.0;// test_frame_fg.at<float>(r, c);
      //test_frame_winner.at<float>(r, c) = test_frame_bg.at<float>(r, c);
      for (int cls = 0; cls < num_classes_; ++cls){
        const float prediction = PredictProb(rgb, cls);
        frame_data[index*frame->NumClassificationChannels() + cls] = prediction;
      }
      for (int cls = num_classes_; cls < frame->NumClassificationChannels(); ++cls){
        frame_data[index*frame->NumClassificationChannels() + cls] = 0;
      }

    }
  }

  return true;

}

Histogram::Histogram() : num_classes_(2){

  bin_divider_ = 256 / histogram_bin_num;

}


size_t Histogram::PredictClass(const cv::Mat &pixel) const {

  float Pf = 1.0f;
  float Pb = 1.0f;

  unsigned char *data = pixel.data;

  for (int c = 0; c < histogram_channel_num; ++c){
    Pf *= (float)foreground_shaft_[c][(int)(floor(data[c]) / bin_divider_)] / (float)fg_area_;
  }

  for (int c = 0; c < histogram_channel_num; ++c){
    Pb *= (float)background_[c][(int)(floor(data[c]) / bin_divider_)] / (float)bg_area_;
  }
  
  if (Pf >= Pb) return 1;
  else if (Pb > Pf) return 0;
  else
    throw std::runtime_error("");

}


float Histogram::PredictProb(const cv::Mat &pixel, const size_t class_index) const {
  
  float P = 1.0f;

  if (class_index == 1){
    for (int c = 0; c < histogram_channel_num; ++c){
      if (pixel.type() == CV_32F){
        P *= (float)foreground_shaft_[c][(int)(floor(pixel.at<float>(c)) / bin_divider_)] / (float)fg_area_;
      }
      else if (pixel.type() == CV_8U){
        P *= (float)foreground_shaft_[c][(int)(floor(pixel.at<unsigned char>(c)) / bin_divider_)] / (float)fg_area_;
      }
      else{
        throw std::runtime_error("");
      }
    }
  }
  else if (class_index == 0){
    for (int c = 0; c < histogram_channel_num; ++c){
      if (pixel.type() == CV_32F){
        P *= (float)background_[c][(int)(floor(pixel.at<float>(c)) / bin_divider_)] / (float)bg_area_;
      }
      else if (pixel.type() == CV_8U){
        P *= (float)background_[c][(int)(floor(pixel.at<unsigned char>(c)) / bin_divider_)] / (float)bg_area_;
      }
      else{
        throw std::runtime_error("");
      }
    }
  }
  else if (class_index == 2){
    for (int c = 0; c < histogram_channel_num; ++c){
      if (pixel.type() == CV_32F){
        P *= (float)foreground_tip_[c][(int)(floor(pixel.at<float>(c)) / bin_divider_)] / (float)fg_area_;
      }
      else if (pixel.type() == CV_8U){
        P *= (float)foreground_tip_[c][(int)(floor(pixel.at<unsigned char>(c)) / bin_divider_)] / (float)fg_area_;
      }
      else{
        throw std::runtime_error("");
      }
    }
  }
  else{
    throw std::runtime_error("Error, bad class index");
  }

  return P;
  
}


void Histogram::TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir){

  throw std::runtime_error("Not implemented!\n");

}

void Histogram::Load(const std::string &url){

  foreground_shaft_.Init();
  foreground_tip_.Init();
  background_.Init();

  std::ifstream ifs(url);
  std::string foreground_file, mask_file;
  ifs >> foreground_file >> mask_file;

  if (!boost::filesystem::exists(boost::filesystem::path(foreground_file)) || !boost::filesystem::exists(boost::filesystem::path(mask_file))){
    throw std::runtime_error("Error, the files are not found!\n");
  }

  cv::Mat foreground_im = cv::imread(foreground_file);
  cv::Mat mask_im = cv::imread(mask_file, 0);

  NDImage nd_image(foreground_im);

  fg_area_ = 0;
  bg_area_ = 0;

  for (int r = 0; r < foreground_im.rows; ++r){
    for (int c = 0; c < foreground_im.cols; ++c){

      cv::Mat pix = nd_image.GetPixelData(r, c);

      if (mask_im.at<unsigned char>(r, c) > 127){
        ++fg_area_;
        for (int hc = 0; hc < histogram_channel_num; ++hc)
          ++foreground_shaft_[hc][(int)floor(pix.at<float>(hc) / bin_divider_)];
          //++foreground_shaft_[hc][(int)floor(foreground_im.at<cv::Vec3b>(r, c)[hc] / bin_divider_)];
      }
      else{
        ++bg_area_;
        for (int hc = 0; hc < histogram_channel_num; ++hc)
          ++background_[hc][(int)floor(pix.at<float>(hc) / bin_divider_)];
          //++background_[hc][(int)floor(foreground_im.at<cv::Vec3b>(r, c)[hc] / bin_divider_)];
      }

    }
  }

}



std::string Histogram::NameAsString() const {
  return "histogram";
}


void Histogram::Save(const std::string &url) const {

  throw std::runtime_error("Not implemented!\n");

}