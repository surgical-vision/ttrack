#ifndef __PRIMITIVES_HPP__
#define __PRIMITIVES_HPP__

#include <cv.h>

namespace ttrk {

  template<int N, typename Precision = double>
  struct Polygon {
    cv::Vec<Precision, 3> vertices[N];
  };

  //typedef Polygon<3,double> Triangle;
  typedef Polygon<4,double> Quadrilateral;

}



#endif
