#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_
#include <string>
#include <vector>
#include "headers.hpp"
#include <unordered_map>

namespace ttrk{

  inline int round(double r){

    return (r > 0.0) ? (int)floor(r + 0.5) : (int)ceil(r - 0.5);

  }



  inline void SAFE_EXIT(){
  #ifdef __linux__
    exit(1);
  #elif defined (_WIN32) || (_WIN64)
    system("pause");
    exit(1);
  #else
    exit(1);
  #endif
  
  }

  inline bool IS_IMAGE(const std::string &extension){
    return (extension == ".png" || extension == ".jpg");
  }

  cv::Mat &ConvertMatSingleToTriple(cv::Mat &im);

  struct hash_Vec{
    size_t operator()(const cv::Vec3b &b) const{
      return std::hash<int>()((int)b[0]) ^ std::hash<int>()((int)b[1]) ^ std::hash<int>()((int)b[2]);
    }
  };

}


#endif
