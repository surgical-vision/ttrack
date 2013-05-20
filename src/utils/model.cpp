#include "../../headers/track/model.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;



MISTool::MISTool(int radius, int height):radius_(radius),height_(height){

  const int precision = 32;

  for(size_t i=0;i<precision;i++){

    points_.push_back( SimplePoint<>( cv::Vec3f(-(float)height_/2*(i>=precision/2) + (float)(height_/2)*(i,precision/2), (float)(radius * cos(i * M_PI/4)), (float)(radius * sin(i * M_PI/4)))));


  }

  for(int i=0;i<precision/2;i++){
    points_[i].AddNeighbour(Wrap(i-1,0,(precision/2)-1));
    points_[i].AddNeighbour(Wrap(i+1,0,(precision/2)-1));
  }

  for(int i=precision/2;i<precision;i++){
    points_[i].AddNeighbour(Wrap(i-1,precision/2,precision-1));
    points_[i].AddNeighbour(Wrap(i+1,precision/2,precision-1));
  }

  for(int i=0;i<precision;i++)
    points_[i].AddNeighbour(Wrap(i+(precision/2),0,precision-1));



}

std::vector<SimplePoint<> > MISTool::Points() const {

  return points_;

}

void MISTool::GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back) const {

  cv::Mat top,bottom;
  top = pose_ * (cv::Mat)cv::Vec4f( (float)height_/2, (float)0.0, (float)0.0, (float)1.0);
  bottom = pose_ * (cv::Mat)cv::Vec3f( -(float)height_/2, (float)0.0, (float)0.0, (float)1.0);

  assert((top.size() == cv::Size(3,1) || top.size() == cv::Size(1,3)) && top.size() == bottom.size());

  cv::Mat central_axis = top - bottom;
  float dpdot = central_axis.dot(central_axis);
  if(dpdot == 0.0) dpdot = 0.0000001;

  cv::Mat t1 = (cv::Mat)ray - ((1.0/(dpdot))*(ray.dot(central_axis))*central_axis);
  



}
