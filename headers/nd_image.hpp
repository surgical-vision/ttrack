#ifndef _ND_IMAGE_HPP_
#define _ND_IMAGE_HPP_
#include "headers.hpp"

class NDImage{

public:

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

protected:
  
  int channels_;
  int rows_;
  int cols_;
  bool bad_;

};

inline size_t NDImage::channels() const {

  return channels_;

}

inline int NDImage::rows() const {

  return rows_;

}

inline int NDImage::cols() const {

  return cols_;  

}

inline bool NDImage::bad() const {

  return bad_;

}



#endif
