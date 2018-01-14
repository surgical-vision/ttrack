#include "../../include/ttrack/detect/baseclassifier.hpp"
#include <cinder/app/App.h>

using namespace ttrk;

cv::Mat BaseClassifier::first_image;
cv::Mat BaseClassifier::first_mask;

BaseClassifier::BaseClassifier(){};
BaseClassifier::~BaseClassifier(){};

void BaseClassifier::GetGabor(const cv::Mat &bgr, cv::Mat &gabor) const {

  std::vector<cv::Mat> filters;
  for (float theta = 0; theta < 3.1415926; theta += 3.1415926 / 16){
    cv::Mat kern = cv::getGaborKernel(cv::Size(31, 31), 4.0, theta, 10.0, 0.5, 0, CV_32F);
    float sum = cv::sum(kern).val[0];
    kern = (1.0 / (1.5 * sum)) * kern;
    filters.push_back(kern.clone());
  }

  cv::Mat bgr_32bit;
  bgr.convertTo(bgr_32bit, CV_32F);
  bgr_32bit *= 1. / 255;

  cv::Mat gray;
  cv::cvtColor(bgr_32bit, gray, CV_BGR2GRAY);

  cv::Mat accum = cv::Mat::zeros(gray.size(), gray.type());
  for (auto kern : filters){

    cv::Mat filtered_image;
    //cv::filter2D(gray, filtered_image, CV_8UC3, kern);
    cv::filter2D(gray, filtered_image, CV_32F, kern);
    accum = cv::max(accum, filtered_image);

  }

  gabor = accum.clone();

}

inline size_t CountBackground(const cv::Mat &m){

  size_t n = 0;
  for (int i = 0; i < m.total(); ++i){

    if (m.at<int>(i) == 0) n++;

  }

  return n;
}


inline size_t CountForeground(const cv::Mat &m){

  size_t n = 0;
  for (int i = 0; i < m.total(); ++i){

    if (m.at<int>(i) > 0) n++;

  }

  return n;
}


void BaseClassifier::UpdateNegativeTrainingDataFromFirstFrame(const cv::Mat &sdf_image){

  size_t num_foreground_in_dataset = CountForeground(training_labels);
  size_t num_background_in_dataset = CountBackground(training_labels);

  int negative_min_limit = -5;
  int negative_max_limit = -30;

  cv::Mat frame_float;
  first_image.convertTo(frame_float, CV_32F);
  frame_float *= 1.0f / 255;
  float *whole_frame_data = (float *)frame_float.data;

  cv::Mat gabor_image;
  GetGabor(frame_float, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(frame_float, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;


  size_t idx = 0;

  cv::Vec4f sample_d;

  for (int r = 0; r < sdf_image.rows; ++r){
    for (int c = 0; c < sdf_image.cols; ++c){
      const float &sdf_val = sdf_image.at<float>(r, c);
      if ((sdf_val > negative_max_limit && sdf_val < negative_min_limit)){

        bool overrun = false;

        if (idx >= training_labels.total()) overrun = true;

        while (!overrun && training_labels.at<int>(idx) != 0) {
          idx++;
          if (idx == training_labels.total()) {
            overrun = true;
            break;
          }
        }

        //if (overrun) { r = sdf_image.rows; break; }

        const int index = r*sdf_image.cols + c;

        if (first_mask.at<float>(r, c) >= 0)
          continue;

        sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
        sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
        sample_d[2] = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
        sample_d[3] = gabor_image_data[index];

        overrun = true;

        if (!overrun){
          for (int d_idx = 0; d_idx < 4; ++d_idx){
            training_data.at<float>(idx, d_idx) = sample_d[d_idx];
          }
        }
        else{
          cv::Mat new_data(1, 4, CV_32FC1);
          for (int d_idx = 0; d_idx < 4; ++d_idx){
            new_data.at<float>(d_idx) = sample_d[d_idx];
          }
          training_data.push_back(new_data);
        }


        //if the original image has the pixel as background
        if (first_mask.at<float>(r, c) < 0){
          if (!overrun){
            training_labels.at<int>(idx) = 0;
          }
          else{
            training_labels.push_back(0);
          }
        }

        else{
          ci::app::console() << "Error, this is an invalid label image in retrain!" << std::endl;
          throw std::runtime_error("");
        }

        idx++;
      }
    }
  }

  if (num_foreground_in_dataset != CountForeground(training_labels)){
    ci::app::console() << "Error, foreground counts don't match!" << std::endl;
    throw std::runtime_error("");
  }

  ResampleData(training_data, training_labels);

  ci::app::console() << "Total training size after update: \nforeground - " << num_foreground_in_dataset << "\nbackground - " << num_background_in_dataset << std::endl;

}

void BaseClassifier::UpdateNegativeTrainingData(const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &label_image){

  size_t num_foreground_in_dataset = CountForeground(training_labels);
  size_t num_background_in_dataset = CountBackground(training_labels);

  int negative_min_limit = -5;
  int negative_max_limit = -30;

  cv::Mat frame_float;
  frame.convertTo(frame_float, CV_32F);
  frame_float *= 1.0f / 255;
  float *whole_frame_data = (float *)frame_float.data;

  cv::Mat gabor_image;
  GetGabor(frame_float, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(frame_float, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;


  size_t idx = 0;

  cv::Vec4f sample_d;

  for (int r = 0; r < sdf_image.rows; ++r){
    for (int c = 0; c < sdf_image.cols; ++c){
      const float &sdf_val = sdf_image.at<float>(r, c);
      if ((sdf_val > negative_max_limit && sdf_val < negative_min_limit)){

        bool overrun = false;

        if (idx >= training_labels.total()) overrun = true;

        while (!overrun && training_labels.at<int>(idx) != 0) {
          idx++;
          if (idx == training_labels.total()) {
            overrun = true; 
            break;
          }
        }

        if (overrun) { r = sdf_image.rows; break; }

        const int index = r*sdf_image.cols + c;

        sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
        sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
        sample_d[2] = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
        sample_d[3] = gabor_image_data[index];

        for (int d_idx = 0; d_idx < 4; ++d_idx)
          training_data.at<float>(idx, d_idx) = sample_d[d_idx];

        if (label_image.at<unsigned char>(r, c) != 0){
          ci::app::console() << "Error, should not have foreground data in here." << std::endl;
          throw std::runtime_error("");
        }

        if (label_image.at<unsigned char>(r, c) == 0){
          training_labels.at<int>(idx) = 0;
        }
        else if (label_image.at<unsigned char>(r, c) == 1){
          training_labels.at<int>(idx) = 1;
        }
        else if (label_image.at<unsigned char>(r, c) == 2){
          training_labels.at<int>(idx) = 2;
        }
        else{
          ci::app::console() << "Error, this is an invalid label image in retrain!" << std::endl;
          throw std::runtime_error("");
        }

        idx++;
      }
    }
  }

  if (num_foreground_in_dataset != CountForeground(training_labels)){
    ci::app::console() << "Error, foreground counts don't match!" << std::endl;
    throw std::runtime_error("");
  }

  //ResampleData(training_data, training_labels);

  ci::app::console() << "Total training size after update: \nforeground - " << num_foreground_in_dataset << "\nbackground - " << num_background_in_dataset << std::endl;

}

void BaseClassifier::LoadPositiveAndNegativeTrainingData__HACK(){

  cv::Mat frame = cv::imread("C:\\Users\\davinci\\data\\processed\\in_vivo_articulated3\\classifier\\data\\image.png");
  cv::Mat mask = cv::imread("C:\\Users\\davinci\\data\\processed\\in_vivo_articulated3\\classifier\\data\\mask.png", 0);

  cv::Mat frame_float;
  frame.convertTo(frame_float, CV_32F);
  frame_float *= 1.0f / 255;
  float *whole_frame_data = (float *)frame_float.data;

  cv::Mat gabor_image;
  GetGabor(frame_float, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(frame_float, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;

  cv::Mat sample_d(1,4, CV_32FC1);
  cv::Mat lab(1, 1, CV_32SC1);
  for (int r = 0; r < mask.rows; ++r){
    for (int c = 0; c < mask.cols; ++c){

      const int index = r*frame.cols + c;

      sample_d.at<float>(0) = (float)(whole_frame_data[(index * 3) + 2]);
      sample_d.at<float>(1) = (float)CIE_lab_data[(index * 3) + 1];
      sample_d.at<float>(2) = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
      sample_d.at<float>(3) = gabor_image_data[index];

      if (mask.at<unsigned char>(r, c) == 127){
        lab.at<int>(0) = 1;
        training_data.push_back(sample_d);
        training_labels.push_back(lab);
      }
      else 
      if (mask.at<unsigned char>(r, c) == 255){
        lab.at<int>(0) = 2;
        training_data.push_back(sample_d);
        training_labels.push_back(lab);
      }

    }    
  }
  
}

void BaseClassifier::UpdateSDF(const cv::Mat &m){

  for (int r = 0; r < first_mask.rows; ++r){
    for (int c = 0; c < first_mask.cols; ++c){
      const float new_sdf_val = m.at<float>(r, c);
      float &old_sdf_val = first_mask.at<float>(r, c);
      if (std::abs(old_sdf_val) > std::abs(new_sdf_val)){
        old_sdf_val = new_sdf_val;
      }
    }
  }

}

void BaseClassifier::LoadPositiveAndNegativeTrainingData2(const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &label_image){

  //turn the whole image into features
  if (first_image.empty()) first_image = frame.clone();
  if (first_mask.empty()) first_mask = sdf_image.clone();
  else UpdateSDF(sdf_image);

  LoadPositiveAndNegativeTrainingData(frame, sdf_image, label_image);
  //store this image

}

void BaseClassifier::LoadPositiveAndNegativeTrainingData(const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &label_image){

  int negative_min_limit = -5;
  int negative_max_limit = -30;

  size_t number_training_samples = 0;
  for (int r = 0; r < sdf_image.rows; ++r){
    for (int c = 0; c < sdf_image.cols; ++c){
      const float &sdf_val = sdf_image.at<float>(r, c);
      if (sdf_val >= 0 || (sdf_val > negative_max_limit && sdf_val < negative_min_limit)){
        number_training_samples++;
      }
    }
  }

  training_data = cv::Mat(number_training_samples, 4, CV_32FC1);
  training_labels = cv::Mat(number_training_samples, 1, CV_32SC1);

  cv::Mat frame_float;
  frame.convertTo(frame_float, CV_32F);
  frame_float *= 1.0f / 255;
  float *whole_frame_data = (float *)frame_float.data;

  cv::Mat gabor_image;
  GetGabor(frame_float, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(frame_float, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;


  size_t idx = 0;

  cv::Vec4f sample_d;

  for (int r = 0; r < sdf_image.rows; ++r){
    for (int c = 0; c < sdf_image.cols; ++c){
      const float &sdf_val = sdf_image.at<float>(r, c);
      if (sdf_val >= 0 || (sdf_val > negative_max_limit && sdf_val < negative_min_limit)){

        const int index = r*sdf_image.cols + c;

        sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
        sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
        sample_d[2] = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
        sample_d[3] = gabor_image_data[index];

        for (int d_idx = 0; d_idx < 4; ++d_idx)
          training_data.at<float>(idx, d_idx) = sample_d[d_idx];

        if (label_image.at<unsigned char>(r, c) == 0){
          training_labels.at<int>(idx) = 0;
        }
        else if (label_image.at<unsigned char>(r, c) == 1){
          training_labels.at<int>(idx) = 1;
        }
        else if (label_image.at<unsigned char>(r, c) == 2){
          training_labels.at<int>(idx) = 2;
        }
        else{
          ci::app::console() << "Error, this is an invalid label image in retrain!" << std::endl;
          throw std::runtime_error("");
        }

        idx++;
      }
    }
  }

  ci::app::console() << "Total training size after init : \nforeground - " << CountForeground(training_labels) << "\nbackground - " << CountBackground(training_labels) << std::endl;

  //static size_t n = 0;
  //if (n<2){
  //  LoadPositiveAndNegativeTrainingData__HACK();
  //  n++;
  //}

  ResampleData(training_data, training_labels);

  ci::app::console() << "Total training size after hack: \nforeground - " << CountForeground(training_labels) << "\nbackground - " << CountBackground(training_labels) << std::endl;
  //

}


void BaseClassifier::ShuffleData(cv::Mat &training_data, cv::Mat &training_labels){

  if (training_data.rows != training_labels.rows) throw std::runtime_error("");

  std::vector <int> seeds;
  for (int cont = 0; cont < training_data.rows; cont++)
    seeds.push_back(cont);

  cv::randShuffle(seeds);

  cv::Mat output_training_data(1, 4, CV_32FC1);
  cv::Mat output_training_labels(1, 1, CV_32SC1);
  for (int cont = 0; cont < training_labels.total(); cont++){

    if (cont == 0){
      cv::Mat f = training_data.row(seeds[cont]);
      for (int k = 0; k < 4; ++k)
        output_training_data.at<float>(k) = f.at<float>(k);

      int ff = training_labels.row(seeds[cont]).at<int>(0);
      output_training_labels.at<int>(0) = ff;

    }
    else{
      output_training_data.push_back(training_data.row(seeds[cont]));
      output_training_labels.push_back(training_labels.row(seeds[cont]));
    }
  }

  training_data = output_training_data;
  training_labels = output_training_labels;

}

void BaseClassifier::ResampleData(cv::Mat &training_data, cv::Mat &training_labels){

  ShuffleData(training_data, training_labels);

  std::vector<size_t> count_of_each_label;
  for (int i = 0; i < 3; ++i){
    count_of_each_label.push_back(0);
  }

  for (int i = 0; i < training_labels.total(); ++i){
    count_of_each_label[training_labels.at<int>(i)]++;
  }

  auto min_value = std::min_element(count_of_each_label.begin(), count_of_each_label.end());

  //if ((total_number_of_foreground_instances * 2) >= count_of_each_label[0]) return;

  cv::Mat resample_training_data;
  cv::Mat resample_training_labels;
  std::vector<size_t> new_count_of_each_label;
  for (int i = 0; i < count_of_each_label.size(); ++i) new_count_of_each_label.push_back(0);

  size_t idx = 0;
  while (1){

    if (idx >= training_data.rows) break;

    bool one_smaller = false;
    for (auto &cnt : new_count_of_each_label) if (cnt <= *min_value) one_smaller = true;
    if (!one_smaller) break;

    for (size_t i = 0; i < new_count_of_each_label.size(); ++i){
      if (training_labels.at<int>(idx) == i && new_count_of_each_label[i] < *min_value){
        resample_training_data.push_back(training_data.row(idx));
        resample_training_labels.push_back(training_labels.row(idx));
        new_count_of_each_label[i]++;
      }
    }

    idx++;
  }

  for (auto &cnt : new_count_of_each_label) if (cnt != *min_value) throw std::runtime_error("Error, resampling failed!");

  training_data = resample_training_data;
  training_labels = resample_training_labels;

  //ShuffleData(training_data, training_labels);

  //zero mean and unit variance the features is not necessary for random forests because features are not compared to each other

}



void BaseClassifier::TrainClassifier(const cv::Mat &frame, const cv::Mat &sdf_image, const cv::Mat &label_image){


  if (training_data.empty())
    LoadPositiveAndNegativeTrainingData2(frame, sdf_image, label_image);
  else
    UpdateNegativeTrainingDataFromFirstFrame(sdf_image);

  TrainClassifier(training_data, training_labels);

}

bool BaseClassifier::ClassifyFrame(boost::shared_ptr<sv::Frame> frame, const cv::Mat &sdf){

  if (frame == nullptr) return false;

  //static cv::Mat detection_frame;

  assert(frame->GetImageROI().type() == CV_8UC3);

  cv::Mat whole_frame = frame->GetImage();

  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  size_t pixel_count = 0;

  cv::Mat &f = frame->GetClassificationMap();
  float *frame_data = (float *)frame->GetClassificationMap().data;
  size_t classification_map_channels = frame->GetClassificationMap().channels();

  //if (!detection_frame.empty()){
  //  f = detection_frame.clone();
  //  return true; 
  //}

  if (frame->GetClassificationMap().depth() != CV_32F){
    ci::app::console() << "Classification map is not 32 F" << std::endl;
    throw std::runtime_error("");
  }

  whole_frame.convertTo(whole_frame, CV_32F);
  whole_frame *= 1.0f / 255;
  float *whole_frame_data = (float *)whole_frame.data;

  cv::Mat gabor_image;
  GetGabor(whole_frame, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(whole_frame, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;

  memset(frame_data, 0, f.total()*f.channels()*sizeof(float));

  cv::Mat sample(4, 1, CV_32FC1);
  float *sample_d = (float *)sample.data;

  for (int r = 0; r < rows; r++){
    for (int c = 0; c < cols; c++){

      if (sdf.at<float>(r, c) < -40) continue;

      const int index = r*cols + c;

      //red
      //a
      //o1
      //gabor

      sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
      sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
      sample_d[2] = 0.5f * ((float)whole_frame_data[(index * 3) + 2] - (float)whole_frame_data[(index * 3) + 1]);
      sample_d[3] = gabor_image_data[index];

      const float prediction = (const float)PredictProb(sample, 1); //need to be between 0 - 255 for later processing stage

      //even though this is redundant, it allows compatibility with multiclass classifiers
      frame_data[index*frame->NumClassificationChannels()] = (1 - prediction); //background
      frame_data[index*frame->NumClassificationChannels() + 1] = (prediction); //foreground

      for (int k = 2; k < frame->NumClassificationChannels(); ++k){

        frame_data[index*frame->NumClassificationChannels() + k] = 0;

      }

      pixel_count += prediction > 0;

    }
  }

  return true;

  //if (pixel_count > (0.02*rows*cols)) return true;
  //else return false;

}