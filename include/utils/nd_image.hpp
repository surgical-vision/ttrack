#ifndef _ND_IMAGE_HPP_
#define _ND_IMAGE_HPP_
#include "../headers.hpp"
#include "../constants.hpp"
#include <map>

namespace ttrk{

  class NDImage{

    /**
     * @class NDImage
     * @brief This is a wrapper class for the multidimensional images. 
     *
     * This takes a single BGR image as input and generates a selection of images in different colour spaces or with strctural descriptors. My indexing a pixel of this 'image', one can receive a cv::Mat of 1xN (where N is the number of features).
     *
     */

  public:

    typedef std::pair<std::string,cv::Mat> NamedImage;

    /**
     * Constructor for the class. A single BGR cv::Mat is passed in and then transformed into the desired colourspaces.
     * @param[in] image The input image.
     */

    NDImage(const cv::Mat &image);
  
    /**
     * Get the number of channels in the image, each channel contains data of type T
     */
    size_t channels() const;

    /**
     * Get the number of rows in the image.
     * @return The number of rows.
     */
    int rows() const;

    /**
     * Get the number of columns in the image.
     * @return The number of columns.
     */
    int cols() const;

    /**
     * Quick sanity check.
     * @return True if image is messed up, false if not.
     */
    bool bad() const;
  
    /**
     * given a pixel in the image, return a 1 x N matrix with the rich feature data for that pixel.
     * @param r The row index.
     * @param c The column index.
     * @return The 1 x N matrix.
     */
    cv::Mat GetPixelData(const int r, const int c) const;
    cv::Mat GetPixelData__old(const int r, const int c) const;
     
    static int channels_;

  protected:

    void ConvertBGR2HS(const cv::Mat &in,cv::Mat &hue,cv::Mat &sat);
    void ConvertBGR2O2(const cv::Mat &in,cv::Mat &out);
    void ConvertBGR2O3(const cv::Mat &in,cv::Mat &out);

    /**
     * Parse the current rgb image and generate the rich feature space from this.
     * @param image A BGR image.
     */
    void SetUpImages__old(const cv::Mat &image);
    void SetUpImages(const cv::Mat &image);
  
    int rows_;
    int cols_;
    bool bad_;
 
    std::map<std::string,cv::Mat> images_;
    cv::Mat images_new_;
  };

  inline int NDImage::rows() const {
    return rows_;
  }

  inline int NDImage::cols() const {
    return cols_;  
  }

  inline bool NDImage::bad() const {
    return bad_;
  }

  inline cv::Mat NDImage::GetPixelData(const int r, const int c) const {

    float *data = (float *)images_new_.data + (r*cols_+c)*channels_;
    cv::Mat return_pix(1,4,CV_32FC1,data);

    return return_pix;

  }

}

#endif
