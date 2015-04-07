#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include <cv.h>
#include <boost/shared_ptr.hpp>

namespace sv {

  template <typename PixelType, int Channels> class Image;
  template <typename PixelType, int Channels> class MonocularImage;
  template <typename PixelType, int Channels> class StereoImage;

  template<typename PixelType, int Channels>
  class __InnerImage {

  public:

    friend class Image<PixelType,Channels>;
    friend class MonocularImage<PixelType,Channels>;
    friend class StereoImage<PixelType,Channels>;

    inline explicit __InnerImage(cv::Mat frame);
    __InnerImage() { frame_ = cv::Mat(0,0,CV_MAKETYPE(cv::DataDepth<PixelType>::value,Channels)); }
    
    void Reset(const cv::Size size);
    cv::Size Size() const;
    int NumChannels() const { return Channels; }
    
    PixelType GetPixelData(const int row, const int col, const int channel) const {
      return *((PixelType *)frame_.data + (row * frame_.cols + col)*Channels + channel);
    }

    cv::Rect frame_roi_;
    cv::Mat frame_;
    PixelType *frame_data_;

  };



  template<typename PixelType, int Channels>
  class Image {

  public:

    struct Pixel_ {
      PixelType data_[Channels];
    };

    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    virtual PixelType *FrameData() { return image_data_.frame_.data; }
    virtual const PixelType *FrameData() const { return image_data_.frame_.data; }

    virtual Pixel operator()(const int r, const int c) const = 0;
    virtual PixelType operator()(const int r, const int c, const int chan) const = 0;  
       
    virtual cv::Mat GetImage() { return image_data_.frame_; } 
    virtual cv::Mat GetImage() const { return image_data_.frame_.clone(); }

    virtual cv::Mat GetImageROI() { return image_data_.frame_(image_data_.frame_roi_); }
    virtual cv::Mat GetImageROI() const { return image_data_.frame_(image_data_.frame_roi_).clone(); }

    virtual cv::Mat GetClassificationMap() { return classification_map_data_.frame_; }
    virtual cv::Mat GetClassificationMap() const { return classification_map_data_.frame_.clone(); }

    virtual cv::Mat GetClassificationMapROI() { return classification_map_data_.frame_(classification_map_data_.frame_roi_); }
    virtual cv::Mat GetClassificationMapROI() const { return classification_map_data_.frame_(classification_map_data_.frame_roi_).clone(); }

    virtual int rows() const { return image_data_.frame_roi_.height; }
    virtual int cols() const { return image_data_.frame_roi_.width; }

    int NumClassificationChannels() const { return classification_map_data_.NumChannels(); }

    cv::Mat GetBinaryClassificationMap(size_t background_index){

      cv::Mat ret = cv::Mat::zeros(classification_map_data_.Size(), CV_8UC1);
      for (int r = 0; r < ret.rows; ++r){
        for (int c = 0; c < ret.cols; ++c){

          ret.at<unsigned char>(r, c) = classification_map_data_.GetPixelData(r, c, 0) < 0.5;

        }
      }

      return ret;

    }

    cv::Mat GetBinaryClassificationMapROI(size_t background_index){

      return GetBinaryClassificationMap(background_index)(classification_map_data_.frame_roi_);

    }

    static cv::Mat GetChannel(cv::Mat multi_channel, int channel_idx){

      std::vector<cv::Mat> channels(multi_channel.channels());
      cv::split(multi_channel, channels);
      if (channel_idx < channels.size()){
        return channels[channel_idx];
      }
      else{
        throw std::runtime_error("Error, bad access!");
      }

    }

  protected:    
    
    __InnerImage<PixelType,Channels> image_data_;
    __InnerImage<float,5> classification_map_data_;
    
  };

  template<typename PixelType, int Channels>
  class MonocularImage : public Image<PixelType,Channels> {

  public:

    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    explicit MonocularImage(cv::Mat frame);
    MonocularImage() { throw(std::runtime_error("Error, Monocular Default Constructor called!\n")); }

    virtual Pixel operator()(const int r, const int c) const;
    virtual PixelType operator()(const int r, const int c, const int chan) const;  

  protected:


  };

  template<typename PixelType, int Channels>
  class StereoImage : public Image<PixelType,Channels> {

  public:
    
    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    explicit StereoImage(cv::Mat stereo_frame) ; 
    StereoImage() { throw(std::runtime_error("ERror, StereoImage Default constructor called!\n")); }

    virtual Pixel operator()(const int r, const int c) const;
    virtual PixelType operator()(const int r, const int c, const int chan) const;  

    cv::Mat GetDisparityMap();
    cv::Mat GetPointCloud();

    cv::Mat GetLeftImage();
    cv::Mat GetRightImage();
    cv::Mat GetLeftImage() const;
    cv::Mat GetRightImage() const;
    cv::Mat GetLeftClassificationMap();
    cv::Mat GetRightClassificationMap();
    cv::Mat GetLeftClassificationMap() const;
    cv::Mat GetRightClassificationMap() const;

    void SwapEyes();

  protected:

    __InnerImage<float,3> point_cloud_data_;
    __InnerImage<short,1> disparity_map_data_;
    
  };

  typedef Image<unsigned char, 3> Frame;
  typedef MonocularImage<unsigned char, 3> MonoFrame;
  typedef StereoImage<unsigned char, 3> StereoFrame;

  /************ __InnerImage ************/

  template<typename PixelType, int Channels>
  inline __InnerImage<PixelType,Channels>::__InnerImage(cv::Mat frame):frame_(frame){
    frame_data_ = (PixelType *)frame_.data;
  }

  template<typename PixelType, int Channels>
  inline void __InnerImage<PixelType,Channels>::Reset(const cv::Size size) { 
    frame_ = cv::Mat(size,CV_MAKETYPE(cv::DataDepth<PixelType>::value,Channels) ); 
  }

  template<typename PixelType, int Channels>
  inline cv::Size __InnerImage<PixelType,Channels>::Size() const { 
    return frame_.data == 0x0 ?  cv::Size(0,0) : frame_.size(); 
  }

  /************ Image ************/


  /************ MonocularImage ************/

  template<typename PixelType, int Channels>
  MonocularImage<PixelType,Channels>::MonocularImage(cv::Mat frame) {
    /*const int width = frame->cols/2;
    const cv::Size size(width,frame->rows);
    image_data_.Reset(size); 
    (*frame)(cv::Range::all(),cv::Range(0,width)).copyTo(*image_data_.frame_);*/
    image_data_.frame_ = cv::Mat(frame.size(),frame.type());
    frame.copyTo(image_data_.frame_);
    image_data_.frame_roi_ = cv::Rect(0,0,frame.cols,frame.rows);
    classification_map_data_.Reset(frame.size());
    classification_map_data_.frame_roi_ = cv::Rect(0,0,frame.cols,frame.rows);

  }

  template<typename PixelType, int Channels>
  typename MonocularImage<PixelType,Channels>::Pixel MonocularImage<PixelType,Channels>::operator()(const int r, const int c) const {

    cv::Vec<PixelType,Channels> src = image_data_.frame_.at<cv::Vec<PixelType,Channels> >(r,c);
    Pixel dst;
    memcpy(&dst, &src, sizeof(PixelType)*Channels);
    return dst;

  }
  
  template<typename PixelType, int Channels>
  PixelType MonocularImage<PixelType,Channels>::operator()(const int r, const int c, const int chan) const {
 
    return (*this)(r,c).data_[chan];
    
  }

  /************ StereoImage ************/

  template<typename PixelType, int Channels>
  StereoImage<PixelType,Channels>::StereoImage(cv::Mat stereo_frame){
    
    image_data_.frame_ = cv::Mat(stereo_frame.size(),stereo_frame.type());
    stereo_frame.copyTo(image_data_.frame_);
    //image_data_.frame_ = stereo_frame;
    classification_map_data_.Reset(stereo_frame.size());
    image_data_.frame_roi_ = cv::Rect(0,0,stereo_frame.cols/2,stereo_frame.rows);
    classification_map_data_.frame_roi_ = cv::Rect(0,0,stereo_frame.cols/2,stereo_frame.rows);
    point_cloud_data_.frame_roi_ = cv::Rect(0,0,stereo_frame.cols/2,stereo_frame.rows);
    disparity_map_data_.frame_roi_ = cv::Rect(0,0,stereo_frame.cols/2,stereo_frame.rows);

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetDisparityMap() {
    if(point_cloud_data_.frame_.data == 0x0) point_cloud_data_.Reset(cv::Size(image_data_.frame_.cols * 2, image_data_.frame_.rows));
    return point_cloud_data_.frame_;
  }
  

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetPointCloud() {
    if(point_cloud_data_.frame_.data == 0x0) point_cloud_data_.Reset(cv::Size(image_data_.frame_.cols * 2, image_data_.frame_.rows));
    return point_cloud_data_.frame_;
  }

  template<typename PixelType, int Channels>
  typename StereoImage<PixelType,Channels>::Pixel StereoImage<PixelType,Channels>::operator()(const int r, const int c) const  {
    Pixel px;
    cv::Vec<PixelType,Channels> ret = image_data_.frame_(image_data_.frame_roi_).at<cv::Vec<PixelType, Channels> >(r,c);
    memcpy(&px,&ret[0],Channels*sizeof(PixelType));
    return px;
  }

  template<typename PixelType, int Channels>
  PixelType StereoImage<PixelType,Channels>::operator()(const int r, const int c, const int chan) const  {
    return (*this)(r,c).data_[chan];
  }

  /*
  template<typename PixelType, int Channels>
  typename MonocularImage<PixelType,Channels>::Pixel MonocularImage<PixelType,Channels>::operator()(const int r, const int c) const {
    Pixel px;
    const int index = (r*this->image_data_.frame_->cols + c)*Channels;
    memcpy(&this->image_data_.frame_data_[index],px.data_,Channels*sizeof(PixelType));
    return px;
  }*/

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetLeftImage(){

    return image_data_.frame_(cv::Rect(0,0,image_data_.frame_.cols/2,image_data_.frame_.rows));

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType, Channels>::GetLeftImage() const {

    return image_data_.frame_(cv::Rect(0, 0, image_data_.frame_.cols / 2, image_data_.frame_.rows)).clone();

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetRightImage(){
    
    return image_data_.frame_(cv::Rect(image_data_.frame_.cols/2,0,image_data_.frame_.cols/2,image_data_.frame_.rows));

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType, Channels>::GetRightImage() const {

    return image_data_.frame_(cv::Rect(image_data_.frame_.cols / 2, 0, image_data_.frame_.cols / 2, image_data_.frame_.rows)).clone();

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType, Channels>::GetLeftClassificationMap() const{

    return classification_map_data_.frame_(cv::Rect(0, 0, image_data_.frame_.cols / 2, image_data_.frame_.rows)).clone();

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType, Channels>::GetRightClassificationMap() const{

    return classification_map_data_.frame_(cv::Rect(image_data_.frame_.cols / 2, 0, image_data_.frame_.cols / 2, image_data_.frame_.rows)).clone();

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetLeftClassificationMap(){

    return classification_map_data_.frame_(cv::Rect(0,0,image_data_.frame_.cols/2,image_data_.frame_.rows));

  }

  template<typename PixelType, int Channels>
  cv::Mat StereoImage<PixelType,Channels>::GetRightClassificationMap(){

    return classification_map_data_.frame_(cv::Rect(image_data_.frame_.cols/2,0,image_data_.frame_.cols/2,image_data_.frame_.rows));

  }

  template<typename PixelType, int Channels>
  void StereoImage<PixelType,Channels>::SwapEyes() {

    const int width = image_data_.frame_.cols/2;
    const int height = image_data_.frame_.rows;
    cv::Rect swapped_roi;
    if(image_data_.frame_roi_.x == 0)
      swapped_roi = cv::Rect(width,0,width,height);
    else if(image_data_.frame_roi_.x == width)
      swapped_roi = cv::Rect(0,0,width,height);
    else
      throw(std::runtime_error("Error, the eyes are messed up!\n"));

    image_data_.frame_roi_ = swapped_roi;
    classification_map_data_.frame_roi_ = swapped_roi;
    point_cloud_data_.frame_roi_ = swapped_roi;
    disparity_map_data_.frame_roi_ = swapped_roi;
    
  }


}

#endif
