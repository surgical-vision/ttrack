#include "../../include/ttrack/detect/multiclass_randomforest.hpp"

using namespace ttrk;

bool MultiClassRandomForest::ClassifyFrame(boost::shared_ptr<sv::Frame> frame){

  if (frame == nullptr) return false;

  assert(frame->GetImageROI().type() == CV_8UC3);
 
  cv::Mat whole_frame = frame->GetImage();
  
  NDImage nd_image(whole_frame);

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

  for (int r = 0; r<rows; r++){
    for (int c = 0; c<cols; c++){

      const int index = r*cols + c;

      cv::Mat pix = nd_image.GetPixelData(r, c);

      bool predicted_class = false;

      //float largest_pred = 0;
      //predicted probability of each class
      for (size_t cls = 0; cls < num_classes_; ++cls){
        const float prediction = (const float)PredictProb(pix, cls);
        frame_data[index*classification_map_channels + cls] = prediction;

        if (cls > 0 && prediction){
          predicted_class = true;
          //if (prediction > largest_pred){
          //largest_pred = prediction;
          //if (cls == 1)
          ///  saveframe.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0);
          //else if (cls == 2)
          //  saveframe.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 0);
        }

      }

      //zero the rest of the values for sanity
      for (int k = num_classes_; k < classification_map_channels; ++k){
        frame_data[index*classification_map_channels + k] = 0;      
      }

      pixel_count += predicted_class;

    }
  }

  //static cv::VideoWriter writer("z:/classification.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, frame->GetClassificationMap().size());

  //if (!writer.isOpened()){
  //  throw(std::runtime_error(""));
  //}

  //writer << saveframe;


  if (pixel_count > (0.02*rows*cols)) return true;
  else return false;

}