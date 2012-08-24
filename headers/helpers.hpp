#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_
#include <string>

inline bool IS_IMAGE(const std::string &extension){
  return (extension == ".png" || extension == ".jpg");
}


#endif
