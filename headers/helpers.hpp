#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_
#include <string>
#include <vector>

inline bool IS_IMAGE(const std::string &extension){
  return (extension == ".png" || extension == ".jpg");
}

void GetImageURLAndSize(const std::string &dir, std::vector<std::string> &urls,
                        size_t &cols, size_t &rows);
                        


#endif
