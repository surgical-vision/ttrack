#ifndef _HELPERS_HPP_
#define _HELPERS_HPP_
#include <string>
#include <vector>
#include "../headers.hpp"
#include <unordered_map>
#include <boost/functional/hash.hpp>

namespace ttrk{


  inline bool PointInImage(const cv::Point &point, const cv::Size &image_size){
    return cv::Rect(0,0,image_size.width,image_size.height).contains(point);
  }

  template<typename Precision> 
  inline bool IsHorizontal(const cv::Point_<Precision> &a, const cv::Point_<Precision> &b){
    return a.y == b.y;
  }

  template<typename Precision> 
  inline bool IsVertical(const cv::Point_<Precision> &a, const cv::Point_<Precision> &b){
    return a.x == b.x;
  }

  template<typename Precision> 
  inline double Gradient(const cv::Point_<Precision> &a, const cv::Point_<Precision> &b){
    return (float)(a.y - b.y)/(a.x - b.x);
  }

  template<typename Precision> 
  inline double YIntersect(const cv::Point_<Precision> &a, const double gradient){
    return a.y - (gradient*a.x);
  }

  inline double norm2(const cv::Vec3f n){
    return n[0]*n[0] + n[1]*n[1] + n[2]*n[2];
  }

  template<typename Precision>
  bool FindGeneralIntersection(const cv::Point_<Precision> &a, const cv::Point_<Precision> &b, const cv::Point_<Precision> &c, const cv::Point_<Precision> &d, cv::Point_<Precision> &intersection){

    const double gradient_ab = Gradient<Precision>(a,b);
    const double yintersect_ab = YIntersect<Precision>(a,gradient_ab);
    const double gradient_cd = Gradient<Precision>(c,d);
    const double yintersect_cd = YIntersect<Precision>(c,gradient_cd);

    double intersection_x = (yintersect_cd - yintersect_ab)/(gradient_ab - gradient_cd);
    double intersection_y = (gradient_ab*intersection_x) + yintersect_ab;
    
    intersection=cv::Point_<Precision>((Precision)intersection_x,(Precision)intersection_y);

    bool i1 = intersection.x >= std::min(a.x,b.x) && intersection_x <= std::max(a.x,b.x) && intersection_y >= std::min(a.y,b.y) && intersection_y <= std::max(a.y,b.y) ;
    bool i2 = intersection.x >= std::min(c.x,d.x) && intersection_x <= std::max(c.x,d.x) && intersection_y >= std::min(c.y,d.y) && intersection_y <= std::max(c.y,d.y) ;

    return i1 && i2;

  }

  template<typename Precision>
  inline bool FindHorizontalIntersection(const cv::Point_<Precision> &horizontal_start, const cv::Point_<Precision> &horizontal_end, const cv::Point_<Precision> &line_start, const cv::Point_<Precision> &line_end, cv::Point_<Precision> &intersection){

    const double gradient = Gradient<Precision>(line_start,line_end);
    double intersect_x = (horizontal_start.y/gradient) - YIntersect<Precision>(line_start,gradient);
    intersection = cv::Point_<Precision>(intersect_x,horizontal_start.y);
    //do max-min checks as the start and end points might be the wrong way around
    return ( intersection.x >= std::min(horizontal_start.x,horizontal_end.x) && intersection.x <= std::max(horizontal_start.x,horizontal_end.x) && intersection.x >= std::min(line_start.x,line_end.x) && intersection.x <= std::max(line_end.x,line_end.x) );

  }

  template<typename Precision>
  inline bool FindVerticalIntersection(const cv::Point_<Precision> &vertical_start, const cv::Point_<Precision> &vertical_end, const cv::Point_<Precision> &line_start, const cv::Point_<Precision> &line_end, cv::Point_<Precision> &intersection){

    const double gradient = Gradient<Precision>(line_start,line_end);
    double intersect_y = gradient*vertical_start.x + YIntersect<Precision>(line_start,gradient);
    intersection = cv::Point_<Precision>(vertical_start.x,intersect_y);
    //do max-min checks as the start and end points might be the wrong way around
    return ( intersection.y >= std::min(vertical_end.y, vertical_start.y) && intersection.y <= std::max(vertical_end.y,vertical_start.y)  && intersection.y >= std::min(line_start.y,line_end.y) && intersection.y <= std::max(line_end.y,line_start.y) );

  }


  template<typename Precision>
  inline bool FindIntersection(const cv::Point_<Precision> &a1, const cv::Point_<Precision> &a2, const cv::Point_<Precision> &b1, const cv::Point_<Precision> &b2, cv::Point_<Precision> &intersection){

    if( IsHorizontal(a1,a2) && !IsHorizontal(b1,b2)){

      return FindHorizontalIntersection<Precision>(a1,a2,b1,b2,intersection);

    }

    if( IsVertical(a1,a2) && !IsVertical(b1,b2) ) {

      return FindVerticalIntersection<Precision>(a1,a2,b1,b2,intersection);

    }

    if( IsHorizontal(b1,b2) && !IsHorizontal(a1,a2) ){

      return FindHorizontalIntersection<Precision>(b1,b2,a1,a2,intersection);

    }

    if( IsVertical(b1,b2) && !IsVertical(a1,a2) ){

      return FindVerticalIntersection<Precision>(b1,b2,a1,a2,intersection);

    }

    return FindGeneralIntersection<Precision>(a1,a2,b1,b2,intersection);

  }


  /**
  * Computes a smoothed heaviside function.
  * @param[in] x The value on the x-axis.
  * @param[in] a The smoothing value.
  * @ The smoothed heaviside value. Will be between 0-1.
  */

  inline double Heaviside(double x, double a=0.3){
    return 1.0/(1.0 + exp(-a*x));
  }

  /**
  * A quick version of a rounding function.
  * @param[in] r The value to round.
  * @return The rounded value.
  */
  inline int round(double r){
    return (r > 0.0) ? (int)floor(r + 0.5) : (int)ceil(r - 0.5);
  }


  /**
  * A wrapper for exiting the program without visual studio closing the cmd prompt. Maybe this can be done with VS?
  */
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

  /**
  * A helper function for finding if a filename is an image by its extension.
  * @param[in] extension The file extension.
  * @return If the function is an image or not.
  */
  inline bool IS_IMAGE(const std::string &extension){
    return (extension == ".png" || extension == ".jpg");
  }

  cv::Mat &ConvertMatSingleToTriple(cv::Mat &im);

  /**
  * @struct hashVec
  * @brief Custon hash function for hashing cv::Vec3b keys.
  */
  struct hashVec{
    size_t operator()(const cv::Vec3b &b) const{
      size_t seed = 0;
      boost::hash_combine(seed,b[0]);
      boost::hash_combine(seed,b[1]);
      boost::hash_combine(seed,b[2]);
      return seed;
      //return std::hash<int>()((int)b[0]) ^ std::hash<int>()((int)b[1]) ^ std::hash<int>()((int)b[2]);
    }
  };


  /** 
  * Wrap allows an iteration of a list/array index with wrapping so when it reaches
  * the max/min value you don't go out of bounds, just back to the start.
  * @param index The index you want to wrap.
  * @param lowerbound The min index.
  * @param upperbound  The max index
  * @return The wrapped index
  */
  inline int Wrap(int x, const int lowerbound, const int upperbound){
    int range_size = upperbound - lowerbound + 1;
    if (x < lowerbound)
      x += range_size * ((lowerbound - x) / range_size + 1);
    return lowerbound + (x - lowerbound) % range_size;
  }

}


#endif
