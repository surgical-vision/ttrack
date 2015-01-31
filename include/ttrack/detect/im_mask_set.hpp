#ifndef _IM_MASK_SET_HPP_
#define _IM_MASK_SET_HPP_
#include <vector>
#include <string>
#include "../headers.hpp"

namespace ttrk{

  class ImageMaskSet {

  public:

    enum LoadType {POSITIVE,NEGATIVE,BOTH};

    LoadType type_;

    ImageMaskSet(LoadType type_,const std::string &root_dir);
    int NumPix() const { return num_pix_; }
    const std::vector<std::string> &image_urls(){ return image_urls_; }
    const std::vector<std::string> &mask_urls(){ return mask_urls_; }


  protected:

    
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

    int GetTrainingSize(const std::vector<std::string> &urls, bool count_only_positive) const;
    int CountNonZero(cv::Mat &im) const;

    std::vector<std::string> image_urls_;
    std::vector<std::string> mask_urls_;
    std::string root_dir_;

    int num_pix_;

  };


}



#endif // _IM_MASK_SET_HPP_
