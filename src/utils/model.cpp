#include "../../headers/track/model.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;



MISTool::MISTool(float radius, float height):radius_(radius),height_(height){ }


std::vector<SimplePoint<> > MISTool::Points(const Pose &pose) const {

  const int precision = 132;
  std::vector< SimplePoint<> > points;
  points.reserve(precision);

  for(size_t i=0;i<precision;i++){

    cv::Vec3f point(-(float)height_/2 + (float)(height_)*(i>=(precision/2)), 
                     (float)(radius_ * cos(i * 4*M_PI/precision)), 
                     (float)(radius_ * sin(i * 4*M_PI/precision)));

   
    point = pose.Transform(point);
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

bool MISTool::GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  //cv::Mat top,bottom;
  cv::Vec3f top((float)height_/2,0.0,0.0);
  top = pose.Transform(top);
  cv::Vec3f bottom((float)-height_/2,0.0,0.0);
  bottom = pose.Transform(bottom);

  cv::Vec3f dP = top - bottom;
  float dPdP = dP.dot(dP);
  if(!dPdP) dPdP = 0.0000000001f;
  cv::Vec3f t_1 = ray - ((ray.dot(dP))/dPdP)*dP;
  cv::Vec3f t_2 = ((bottom.dot(dP))/dPdP)*dP - bottom;

  float a = t_1.dot(t_1);
  float b = 2*t_1.dot(t_2);
  float c = t_2.dot(t_2) - radius_*radius_;

  float det = (b*b) - (4*a*c);

  if(det <= 0) 
    return false;
  
  if(!a) a = 0.00000000001f;

  float s2 = (-b - sqrt(det))/(2*a);
  float s1 = (-b + sqrt(det))/(2*a);
  
  float alp_1 = (s1*((float)ray.dot(dP)) - (float)(bottom.dot(dP)))/(dPdP);
  float alp_2 = (s2*((float)ray.dot(dP)) - (float)(bottom.dot(dP)))/(dPdP);
  
  
  
  if(dP.dot(dP)){
    cv::Vec3f tdP = dP;
    //dP = cv::normalize(dP);
    cv::normalize(tdP,dP);
  }
  
  //check the intersections with the front of the cylinder
  bool front_intersection = false;
  if(alp_1 >= 0.0 && alp_1 <= 1.0){ //intersection w/ side of cylinder
    front = s1*ray;
    front_intersection = true;
  }else{ //if not there then check for intersection with circular head of cylinder
    front_intersection = CircleIntersection(bottom,dP,ray,radius_,front);
    if(!front_intersection)
      front_intersection = CircleIntersection(top,dP,ray,radius_,front);
  }
  if(!front_intersection) return false;

  //check the intersection with the back of the cylinder
  bool back_intersection = false;
  if(alp_2 >= 0.0 && alp_2 <= 1.0){
    back = s2*ray;
    back_intersection = true;
  }else{
    back_intersection = CircleIntersection(bottom,dP,ray,radius_,back);
    if(!back_intersection)
      back_intersection = CircleIntersection(bottom,dP,ray,radius_,back);
  }

  //if we have the tangent ray then just hack this
  if(front_intersection && !back_intersection)
    back = front;// + cv::Vec3f(0.01,0.01,0.01);

  //sanity check
  if(front[0]!=front[0]){
    throw(std::runtime_error("Error, cylinder intersection routine is giving nans!\n"));

  }

  //another sanity check
  if(front[2] > back[2]){
    for(int c=0;c<3;c++)
      std::swap(front[c],back[c]);
  }

  return front_intersection;
}

bool MISTool::CircleIntersection(const cv::Vec3f &A, const cv::Vec3f &n, const cv::Vec3f &d, const float R, cv::Vec3f &intersection) const{
  
  assert(sqrt(n.dot(n)) >= 0.99 && sqrt(n.dot(n)) <= 1.01); //assert normalized
  float dn = d.dot(n);

  if(dn == (float)0.0) dn = (float)0.000000000001;

  float mu = (A.dot(n))/(dn);

  cv::Vec3f to_center = A - (mu*d);
  if(sqrt(to_center.dot(to_center)) > R)
    return false;
  else
    intersection = mu*d;
  return true;

}
