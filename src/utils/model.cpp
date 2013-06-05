#include "../../headers/track/model.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;



MISTool::MISTool(int radius, int height):radius_(radius),height_(height){ }


std::vector<SimplePoint<> > MISTool::Points() const {


  const int precision = 32;
  std::vector< SimplePoint<> > points;
  points.reserve(precision);

  for(size_t i=0;i<precision;i++){

    cv::Vec3f point(-(float)height_/2*(i>=precision/2) + (float)(height_/2)*(i,precision/2), (float)(radius_ * cos(i * M_PI/4)), (float)(radius_ * sin(i * M_PI/4)));

    //transform point

    points.push_back( SimplePoint<>(point) );


  }

  for(int i=0;i<precision/2;i++){
    points[i].AddNeighbour(Wrap(i-1,0,(precision/2)-1));
    points[i].AddNeighbour(Wrap(i+1,0,(precision/2)-1));
  }

  for(int i=precision/2;i<precision;i++){
    points[i].AddNeighbour(Wrap(i-1,precision/2,precision-1));
    points[i].AddNeighbour(Wrap(i+1,precision/2,precision-1));
  }

  for(int i=0;i<precision;i++)
    points[i].AddNeighbour(Wrap(i+(precision/2),0,precision-1));


  return points;
}


void MISTool::GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  cv::Mat top,bottom;
  //top = pose_ * (cv::Mat)cv::Vec4f( (float)height_/2, (float)0.0, (float)0.0, (float)1.0);
  //bottom = pose_ * (cv::Mat)cv::Vec3f( -(float)height_/2, (float)0.0, (float)0.0, (float)1.0);

  assert((top.size() == cv::Size(3,1) || top.size() == cv::Size(1,3)) && top.size() == bottom.size());

  cv::Mat central_axis = top - bottom;
  double dpdot = central_axis.dot(central_axis);
  if(dpdot == 0.0f) dpdot = 0.0000001;

  cv::Mat t1 = (cv::Mat)ray - ((1.0/(dpdot))*(ray.dot(central_axis))*central_axis);
  



}
