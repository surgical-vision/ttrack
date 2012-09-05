#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_
#include <string>
#include <vector>

inline bool IS_IMAGE(const std::string &extension){
  return (extension == ".png" || extension == ".jpg");
}

/**
 * GetImageURL searches through a root directory pushing any file with the extension .png or .jpg into the urls vector. The urls vector contains the full pathname.
 * @param root_url The root directory.
 * @param urls The vector containing the full pathname of each image.
 */

void GetImageURL(const std::string &root_url, std::vector<std::string> &urls);
                        
/**
 * This function returns the size of the contribution to the training vector.
 * @param urls A vector of urls for the the files.
 * @param num_pix The number of pixels to add.
 * @param positive If the image is a positive training example. If it is, the only add the pixels which correspond to positives in the mask. If it is not then add the entire image as negative images contain no positive pixels.
 */

void GetTrainingSize(const std::vector<std::string> &urls, size_t &num_pix, const bool positive);

int CountNonZero(cv::Mat &im);

#endif
