#ifndef _ND_IMAGE_HPP_
#define _ND_IMAGE_HPP_

#include <map>

#include "../headers.hpp"
#include "../constants.hpp"

const int num_channels = 4; /**< Number of channels we are using. */

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
    size_t channels() const { return num_channels; }

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
     
    /**
    * Get the colorspace image from the set.
    * @param[in] The name of the image.
    * @return The colorspace image.
    */
    cv::Mat GetImage(const std::string &name) { return images_[name]; }

  protected:

    /**
    * Convert the BGR image to Hue and Saturation.
    * @param[in] in The input image.
    * @param[out] hue The hue image.
    * @param[out] sat The saturation image.
    */
    void ConvertBGR2HS(const cv::Mat &in,cv::Mat &hue,cv::Mat &sat);
    
    /**
    * Convert the BGR image to Opponent 1 color space.
    * @param[in] in The input image.
    * @param[out] out The opponent 1 image.
    */
    void ConvertBGR2O1(const cv::Mat &in,cv::Mat &out);

    /**
    * Convert the BGR image to Opponent 2 color space.
    * @param[in] in The input image.
    * @param[out] out The opponent 2 image.
    */
    void ConvertBGR2O2(const cv::Mat &in,cv::Mat &out);

    /**
     * Parse the current rgb image and generate the rich feature space from this.
     * @param image A BGR image.
     */
    void SetUpImages(const cv::Mat &image);
  
    int rows_; /**< The number of rows in the image. */
    int cols_; /**< The number of cols in the image. */
    bool bad_; /**< Sanity test for the image. */
 
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

    float *data = (float *)images_new_.data + (r*cols_ + c)*channels();
    cv::Mat return_pix(1,4,CV_32FC1,data);

    return return_pix;

  }

}

#endif
