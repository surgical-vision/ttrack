#include "../headers/model.hpp"
#include "../headers/helpers.hpp"

using namespace ttrk;



MISTool::MISTool(int radius, int height):radius_(radius),height_(height){

  const int precision = 32;

  for(size_t i=0;i<precision;i++){

    points_.push_back( SimplePoint<>( cv::Vec3f(-(float)height_/2*(i>=precision/2) + (float)(height_/2)*(i,precision/2), radius * cos(i * M_PI/4), radius * sin(i * M_PI/4))));


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
