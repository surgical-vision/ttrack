#include "../headers/supportvectormachine.hpp"

using namespace ttrk;

size_t SupportVectorMachine::PredictClass(const cv::Vec3b &pixel) const {

  return 1;

}

float SupportVectorMachine::PredictProb(const cv::Vec3b &pixel, const size_t class_index) const {

  return 1.0f;

}


void SupportVectorMachine::Load(const std::string &url){



}

void SupportVectorMachine::TrainClassifier(const cv::Mat &training_data,const cv::Mat &labels, const std::string &root_dir){




}
