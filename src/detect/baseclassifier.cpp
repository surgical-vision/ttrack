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

  size_t pixel_count = 0;

  if (frame->NumClassificationChannels() < 2) throw(std::runtime_error("Error, there must be 2 or more channels!"));

  float *frame_data = (float *)frame->GetClassificationMap().data;
  for (int r = 0; r<rows; r++){
    for (int c = 0; c<cols; c++){

      const int index = r*cols + c;

      cv::Mat pix = nd_image.GetPixelData(r, c);

      const float prediction = (const float)PredictProb(pix, 1); //need to be between 0 - 255 for later processing stage

      //even though this is redundant, it allows compatibility with multiclass classifiers
      frame_data[index*frame->NumClassificationChannels()] = (1 - prediction); //background
      frame_data[index*frame->NumClassificationChannels() + 1] = (prediction); //foreground

      for (int k = 2; k < frame->NumClassificationChannels(); ++k){

        frame_data[index*frame->NumClassificationChannels() + k] = 0;

      }

      pixel_count += prediction > 0;

    }
  }

  if (pixel_count > (0.02*rows*cols)) return true;
  else return false;

}