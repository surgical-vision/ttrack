#ifndef Triangle_h_
#define Triangle_h_

#include <cmath>
#include "Object.h"
#include <utility>

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
