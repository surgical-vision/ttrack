#ifndef Sphere_h_
#define Sphere_h_

#include <cmath>
#include "Object.h"
#include <utility>
/*
#include "../../../model.hpp"

bool GetIntersectionPlane(const std::vector<cv::Vec3f >&points_in_plane, const cv::Vec3f &ray, cv::Vec3f &intersection) const {

//find normal to plane and flip for direction with ray
if(points_in_plane.size() < 3) throw(std::runtime_error("Error, must be at least 3 points to use this function!\n"));

cv::Vec3f a,b,c;
a = points_in_plane[0];
b = points_in_plane[1];
c = points_in_plane[2];

const cv::Vec3f ba = a - b;
const cv::Vec3f bc = c - b;

cv::Vec3f normal = ba.cross(bc);

//cv::normalize(normal,n_normal);

float dot = normal.dot(ray);
if(dot < 0) normal *= -1; //normal should point towards ray

const double denom = ray.dot(normal);
if(denom == 0.0) return false; //only if parallel

double distance_along_ray =  b.dot(normal)/denom;
intersection = distance_along_ray * ray;
return true; 

}


bool IntersectionInConvexHull(const std::vector<cv::Vec3f>&convex_hull, const cv::Vec3f &intersection) const {

if(convex_hull.size() < 3) throw(std::runtime_error("Error, must be at least 3 points to use this function!\n"));

cv::Vec3f mean_point(0,0,0);
for( auto hull_pt = convex_hull.begin(); hull_pt != convex_hull.end(); hull_pt++ ){
mean_point += hull_pt;
}

int c = convex_hull.size();
while( mean_point/c == intersection ) { //ensure that the mean point is not the same coordiantes as point
for( auto hull_pt = convex_hull.begin()+2; hull_pt != convex_hull.end(); hull_pt++ ){
mean_point += hull_pt;
}
}
mean_point /= c;


cv::Vec3f line_segment =  mean_point - intersection; 
line_segment *= 5000;  //make the line huge

std::vector<std::pair<cv::Vec3f,cv::Vec3f> >edges;
for(auto point = convex_hull.begin(); point != convex_hull.end() - 1; point++ ){

cv::Vec3f a = *point;
cv::Vec3f b = *(point + 1);

edges.push_back( std::make_pair<>(a,b) );

}
edges.push_back( std::make_pair<>(*(convex_hull.end()-1),*convex_hull.begin()) );

int num_intersections = 0;

for(auto line = edges.begin(); line != edges.end(); line++){
cv::Point2f a(line->first[0]/line->first[2],line->first[1]/line->first[2]),
b(line->second[0]/line->second[2],line->second[1]/line->second[2]),
c(intersection[0]/intersection[2],intersection[1]/intersection[2]),
d(line_segment[0]/line_segment[2],line_segment[1]/line_segment[2]);
num_intersections += FindIntersection<float>(a,b,c,d, cv::Point2f());
}

return num_intersections == 1; //if both mean_point and point are inside polygon this will equal 

}
*/

struct PluckerCoords {
  PluckerCoords(const Vector3 &a, const Vector3 &b){
    coords_ = std::make_pair<>(a,b);
  }
  PluckerCoords(){
    coords_ = std::make_pair<>(Vector3(0,0,0),Vector3(0,0,0));
  }
  double operator[](int n) const {
    if(n < 3)
      return coords_.first[n];
    else
      return coords_.second[n-3];
  }
  std::pair<Vector3,Vector3> coords_;
};


//! For the purposes of demonstrating the BVH, a simple sphere
struct Triangle : public Object {
  Vector3 v1,v2,v3;
  Vector3 center; 
  Vector3 bb_min;
  Vector3 bb_max;
  PluckerCoords plucker_1;
  PluckerCoords plucker_2;
  PluckerCoords plucker_3;
  BBox bbox;
  Triangle(const Vector3 &v1,const Vector3 &v2,const Vector3 &v3)
    : v1(v1), v2(v2), v3(v3) { 
      center = (v1 + v2 + v3)/3;
      bb_min = Vector3(
        std::min(v1[0],std::min(v2[0],v3[0])),
        std::min(v1[1],std::min(v2[1],v3[1])),
        std::min(v1[2],std::min(v2[2],v3[2]))
        );

      bb_max = Vector3(
        std::max(v1[0],std::max(v2[0],v3[0])),
        std::max(v1[1],std::max(v2[1],v3[1])),
        std::max(v1[2],std::max(v2[2],v3[2]))
        );
      bbox = BBox(bb_min,bb_max);
      plucker_1 = getPlucker(v1,v2);
      plucker_2 = getPlucker(v2,v3);
      plucker_3 = getPlucker(v3,v1);

  }

  bool getIntersection(const Ray& ray, IntersectionInfo* I) const {
        
    PluckerCoords plucker_ray = getPlucker(ray.o,ray.d);
    const double s1 = getPluckerInnerProduct(plucker_ray,plucker_1);
    const double s2 = getPluckerInnerProduct(plucker_ray,plucker_2);

    const double length = std::sqrt(center[0]*center[0] + center[1]*center[1] + center[2]*center[2]);

    if(s1 > 0 && s2 > 0){ //counter-clockwise around the ray
      if(getPluckerInnerProduct(plucker_ray,plucker_3) > 0){
        I->object = this;
        I->t = length;
        return true;
      }
    }
    if(s1 < 0 && s2 < 0){ //clockwise around the ray
      if(getPluckerInnerProduct(plucker_ray,plucker_3) < 0){
        I->object = this;
        I->t = length;
        return true;
      }
    }
    //intersects at least 2 edges
    if(s1 == 0 && s2 == 0){
      I->object = this;
      I->t = length;
      return true;
    }
    return true;
    if(s1 == 0 || s2 == 0){
      if(getPluckerInnerProduct(plucker_ray,plucker_3) == 0){
        I->object = this;
        I->t = length;
        return true;
      }
    }

    return false;
  }


  /*
  Given a point P and direction L, the Plu\"{c}ker Coordinates are given as {L,LxP}
  */

  PluckerCoords getPlucker(const Vector3 v1, const Vector3 &v2) const{
    return PluckerCoords(v2-v1,v2^v1);
  }

  double getPluckerInnerProduct(const PluckerCoords &p1, const PluckerCoords &p2) const{
    return (p1.coords_.first * p2.coords_.second) + (p2.coords_.first * p1.coords_.second);
  }

  Vector3 getNormal(const IntersectionInfo& I) const {
    return normalize(I.hit - center);
  }

  BBox getBBox() const { 
    return bbox;              
  }

  Vector3 getCentroid() const {
    return center;
  }

};

#endif
