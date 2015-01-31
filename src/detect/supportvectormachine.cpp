#include "../../include/ttrack/detect/supportvectormachine.hpp"

using namespace ttrk;

size_t SupportVectorMachine::PredictClass(const cv::Mat &pixel) const {

  return 1;

}

float SupportVectorMachine::PredictProb(const cv::Mat &pixel, const size_t class_index) const {

  return 1.0f;

}


void SupportVectorMachine::Load(const std::string &url){



}

void SupportVectorMachine::TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir){

}

std::string SupportVectorMachine::NameAsString() const {
  return "support_vector_machine";
}

void SupportVectorMachine::Save(const std::string &url) const {


}
