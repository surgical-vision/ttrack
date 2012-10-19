#include "../headers/randomforest.hpp"

using namespace ttrk;

size_t RandomForest::PredictClass(const cv::Vec3b &pixel) const {

  return 1;

}


float RandomForest::PredictProb(const cv::Vec3b &pixel, const size_t class_index) const {

  return 1.0f;

}


void RandomForest::Load(const std::string &url){



}
