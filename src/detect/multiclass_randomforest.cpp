#include "../../include/ttrack/detect/multiclass_randomforest.hpp"

using namespace ttrk;

void MultiClassRandomForest::PredictProb(const cv::Mat &sample, float *probabilities, const int size){

  memset(probabilities, 0, sizeof(float)*size);

  for (int n = 0; n<forest_.get_tree_count(); n++){

    CvForestTree *tree = forest_.get_tree(n);
    CvDTreeNode *thing = tree->predict(sample);
    
    probabilities[(size_t)(tree->predict(sample)->value)]+=1.0f;

  }

  for (int i = 0; i < size; ++i) probabilities[i] = probabilities[i] / forest_.get_tree_count();

}


bool MultiClassRandomForest::ClassifyFrame(boost::shared_ptr<sv::Frame> frame, const cv::Mat &sdf_image){

  if (frame == nullptr) return false;

  assert(frame->GetImageROI().type() == CV_8UC3);
 
  cv::Mat whole_frame = frame->GetImage();

  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  size_t pixel_count = 0;

  cv::Mat &f = frame->GetClassificationMap();
  float *frame_data = (float *)frame->GetClassificationMap().data;
  size_t classification_map_channels = frame->GetClassificationMap().channels();
  
  //cv::Mat saveframe = cv::Mat::zeros(frame->GetClassificationMap().size(), CV_8UC3);

  if (num_classes_ > classification_map_channels){
    throw std::runtime_error("");
  }

  if (frame->GetClassificationMap().depth() != CV_32F){
    throw std::runtime_error("");
  }

  //cv::Mat hsv; 

  whole_frame.convertTo(whole_frame, CV_32F);
  whole_frame *= 1.0f / 255;
  float *whole_frame_data = (float *)whole_frame.data;

  //cv::cvtColor(whole_frame, hsv, CV_BGR2HSV);
  //float *hsv_data = (float *)hsv.data;

  cv::Mat gabor_image;
  GetGabor(whole_frame, gabor_image);
  float *gabor_image_data = (float *)gabor_image.data;
  cv::Mat CIE_lab;
  cv::cvtColor(whole_frame, CIE_lab, CV_BGR2Lab);
  float *CIE_lab_data = (float *)CIE_lab.data;

  //memset(frame_data, 0, f.total()*f.channels()*sizeof(float));
  
  cv::Mat sample(4, 1, CV_32FC1);
  float *sample_d = (float *)sample.data;

  for (int r = 0; r < rows; r++){
    for (int c = 0; c < cols; c++){

      if (sdf_image.at<float>(r, c) < -70) continue;

      const int index = r*cols + c;
     
      //red
      //a
      //o1
      //gabor

      sample_d[0] = (float)(whole_frame_data[(index * 3) + 2]);
      //sample_d[1] = (float)(hsv_data[(index * 3) + 1]);
      sample_d[1] = (float)CIE_lab_data[(index * 3) + 1];
      sample_d[2] = 0.5f * ((float)whole_frame_data[(index*3) + 2] - (float)whole_frame_data[(index*3) + 1]);
      //sample_d[3] = (0.5f*(float)whole_frame_data[(index * 3)]) - (0.25f*((float)whole_frame_data[(index * 3) + 2] + (float)whole_frame_data[(index * 3) + 1]));
      sample_d[3] = gabor_image_data[index];
      PredictProb(sample, &frame_data[index*classification_map_channels], num_classes_);

    }
  }

  return true;



}