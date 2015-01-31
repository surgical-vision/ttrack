#include "../../include/ttrack/detect/baseclassifier.hpp"

using namespace ttrk;

BaseClassifier::BaseClassifier(){};
BaseClassifier::~BaseClassifier(){};

bool BaseClassifier::ClassifyFrame(boost::shared_ptr<sv::Frame> frame){

  if (frame == nullptr) return false;

  assert(frame->GetImageROI().type() == CV_8UC3);

  cv::Mat whole_frame = frame->GetImage();
  NDImage nd_image(whole_frame);
  const int rows = whole_frame.rows;
  const int cols = whole_frame.cols;

  static size_t frame_count = 0;

  size_t pixel_count = 0;

  //unsigned char *frame_data = (unsigned char *)frame_->PtrToClassificationMap()->data;
  float *frame_data = (float *)frame->GetClassificationMap().data;
  for (int r = 0; r<rows; r++){
    for (int c = 0; c<cols; c++){

      const int index = r*cols + c;

      cv::Mat pix = nd_image.GetPixelData(r, c);

      float hue = pix.at<float>(0);
      float sat = pix.at<float>(1);
      float o1 = pix.at<float>(2);
      float o2 = pix.at<float>(3);

      //const unsigned char prediction = (unsigned char)255*classifier_->PredictClass(pix);
      const float prediction = (const float)PredictProb(pix, 1); //need to be between 0 - 255 for later processing stage

      frame_data[index*frame->NumClassificationChannels()] = prediction;

      for (int k = 1; k < frame->NumClassificationChannels(); ++k){

        frame_data[index*frame->NumClassificationChannels() + k] = 0;

      }

      pixel_count += prediction > 0;

    }
  }

  if (pixel_count > (0.02*rows*cols)) return true;
  else return false;

}