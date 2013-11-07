#include "../../headers/track/model.hpp"
#include "../../headers/utils/helpers.hpp"
#include <fstream>
using namespace ttrk;



MISTool::MISTool(float radius, float height):radius_(radius),height_(height){
  radius_fraction_ = 0.8;
  height_fraction_ = 1.28;
  radius_tip_ = 2;
  height_tip_ = height_ + 18;
  height_curve_ = height_ + 9;
  angle_curve_ = M_PI/9;
  //tip_offset_ = 0.3;
}

cv::Vec3f MISTool::GetTrackedPoint() const {

  return cv::Vec3f (-(float)height_/2 + (float)(height_tip_), 
                     (float) - 0.15*radius_*2,
                     0);

}

std::vector<SimplePoint<> > MISTool::Points(const Pose &pose) const {

  const int precision = 132;
  std::vector< SimplePoint<> > points;
  points.reserve(precision);


  //////////////////////////////
  //shaft 
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

  //////////////////////////////
  //straight tip
  std::vector<SimplePoint<> > clasper_points_bottom;
  for(size_t i=0;i<precision;i++){
    //needs to start from height_ so the bases line up 
    cv::Vec3f point(-(float)height_/2 + (float)(height_curve_)*(i>=(precision/2)), 
                     (float)(radius_tip_ * cos(i * 4*M_PI/precision)) - (radius_ - radius_tip_),
                     (float)(radius_tip_ * sin(i * 4*M_PI/precision)));
    point = pose.Transform(point);
    //transform point
    clasper_points_bottom.push_back( SimplePoint<>(point) );


  }

  for(int i=0;i<precision/2;i++){
    clasper_points_bottom[i].AddNeighbour(precision+Wrap(i-1,0,(precision/2)-1));
    clasper_points_bottom[i].AddNeighbour(precision+Wrap(i+1,0,(precision/2)-1));
  }

  for(int i=precision/2;i<precision;i++){
    clasper_points_bottom[i].AddNeighbour(precision+Wrap(i-1,precision/2,precision-1));
    clasper_points_bottom[i].AddNeighbour(precision+Wrap(i+1,precision/2,precision-1));
  }

  for(int i=0;i<precision;i++)
    clasper_points_bottom[i].AddNeighbour(precision+Wrap(i+(precision/2),0,precision-1));
  
  points.insert(points.end(),clasper_points_bottom.begin(),clasper_points_bottom.end());

  std::vector< std::vector<SimplePoint<> > > clasper_top_polygons = GetClasperPolygons(pose);
  int num_points = precision;
  for(auto i=clasper_top_polygons.begin(); i != clasper_top_polygons.end() ; i++){
    for(auto point=i->begin();point!=i->end();point++){
      for(auto neighbor=point->neighbours_.begin();neighbor!=point->neighbours_.end();neighbor++){
        *neighbor = *neighbor + points.size();
      }
    }
    points.insert(points.end(),i->begin(),i->end());
  }

  return points;
}

void AddPolygonPoints(std::vector<SimplePoint<> >&points, const Pose &rigid_t, const Pose &shift_head, const Pose &curve_head){
    
  for(size_t i=0;i<points.size();i++){

    points[i].AddNeighbour(Wrap(i-1,0,points.size()-1));
    points[i].AddNeighbour(Wrap(i+1,0,points.size()-1));

    if(points[i].vertex_[0] < 0) {
      points[i].vertex_ = rigid_t.Transform(shift_head.Transform(points[i].vertex_));
      continue;
    }
    points[i].vertex_ = curve_head.Transform(points[i].vertex_);
    points[i].vertex_ = rigid_t.Transform(shift_head.Transform(points[i].vertex_));

  }
  
  

}

std::vector<std::vector<SimplePoint<> > > MISTool::GetClasperPolygons(const Pose &pose) const {

  //////////////////////////////
  //curved tip
  std::vector<std::vector<SimplePoint<> > > clasper_top_polygons; //there will be a few repeated points
  std::vector<SimplePoint<> > polygon_A; //large segment front
  std::vector<SimplePoint<> > polygon_B; //large segment back
  std::vector<SimplePoint<> > polygon_C; //bottom
  std::vector<SimplePoint<> > polygon_D; //top
  std::vector<SimplePoint<> > polygon_E; //face
  std::vector<SimplePoint<> > polygon_F; //angled bit

  //THE ORDERING OF ADDING THESE POINTS IS ESSENTIAL - IF CHANGES ARE MADE, THE NEIGHBOR ADDING & INTERSECTION TESTS NEED TO BE UPDATED
  sv::Quaternion q = sv::Quaternion::FromVectorToVector(cv::Vec3f(1,0,0),cv::Vec3f(0.9,0.1,0.20));
  Pose rotate_pose(cv::Vec3f(0,0,0),q);
  Pose translate_to_front(cv::Vec3f(-(float)height_/2 + (float)(height_curve_) + (height_tip_-height_curve_)/2, 
                                    - 0.15*radius_*2, //need to shift center down
                                     0),
                          sv::Quaternion());

  //four tip points which are rotated slightly off axis
  const float top_fraction = 0.6;
  const float height_fraction = 0.0;
  
  polygon_A.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , -(radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_A.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) * top_fraction  , -(radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_A.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, (radius_tip_/sqrt(2.0)))));
  polygon_A.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_A.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
 
  AddPolygonPoints(polygon_A,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_A);

  polygon_B.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , -(radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_B.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) * top_fraction  , -(radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_B.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, -(radius_tip_/sqrt(2.0)))));
  polygon_B.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_B.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  
  AddPolygonPoints(polygon_B,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_B);
  
  polygon_C.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2)* top_fraction , -(radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_C.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , -(radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_C.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , -(radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_C.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2)* top_fraction, -(radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  
  AddPolygonPoints(polygon_C,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_C);

  polygon_D.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_D.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_D.push_back(SimplePoint<>(cv::Point3f( -((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_D.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
 
  AddPolygonPoints(polygon_D,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_D);

  polygon_E.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, (radius_tip_/sqrt(2.0)))));
  polygon_E.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));  
  polygon_E.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  polygon_E.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, -(radius_tip_/sqrt(2.0)))));

  AddPolygonPoints(polygon_E,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_E);

  polygon_F.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) * top_fraction  , -(radius_tip_/sqrt(2.0)) , (radius_tip_/sqrt(2.0)))));
  polygon_F.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, (radius_tip_/sqrt(2.0)))));
  polygon_F.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) , (radius_tip_/sqrt(2.0)) * height_fraction, -(radius_tip_/sqrt(2.0)))));
  polygon_F.push_back(SimplePoint<>(cv::Point3f( ((height_tip_-height_curve_)/2) * top_fraction  , -(radius_tip_/sqrt(2.0)) , -(radius_tip_/sqrt(2.0)))));
  
  AddPolygonPoints(polygon_F,pose,translate_to_front,rotate_pose);
  clasper_top_polygons.push_back(polygon_F);

  return clasper_top_polygons;
}

bool MISTool::GetIntersectionPlane(const std::vector<SimplePoint<> >&points_in_plane, const cv::Vec3f &ray, cv::Vec3f &intersection) const {

  //find normal to plane and flip for direction with ray
  if(points_in_plane.size() < 3) throw(std::runtime_error("Error, must be at least 3 points to use this function!\n"));

  cv::Vec3f a,b,c;
  a = points_in_plane[0].vertex_;
  b = points_in_plane[1].vertex_;
  c = points_in_plane[2].vertex_;

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


bool MISTool::IntersectionInConvexHull(const std::vector<SimplePoint<> >&convex_hull, const cv::Vec3f &intersection) const {

  if(convex_hull.size() < 3) throw(std::runtime_error("Error, must be at least 3 points to use this function!\n"));

  cv::Vec3f mean_point(0,0,0);
  for( auto hull_pt = convex_hull.begin(); hull_pt != convex_hull.end(); hull_pt++ ){
      mean_point += hull_pt->vertex_;
  }
  
  int c = convex_hull.size();
  while( mean_point/c == intersection ) { //ensure that the mean point is not the same coordiantes as point
    for( auto hull_pt = convex_hull.begin()+2; hull_pt != convex_hull.end(); hull_pt++ ){
      mean_point += hull_pt->vertex_;
    }
  }
  mean_point /= c;
  
  
  cv::Vec3f line_segment =  mean_point - intersection; 
  line_segment *= 5000;  //make the line huge
  
  std::vector<std::pair<cv::Vec3f,cv::Vec3f> >edges;
  for(auto point = convex_hull.begin(); point != convex_hull.end() - 1; point++ ){

    cv::Vec3f a = point->vertex_;
    cv::Vec3f b = (point + 1)->vertex_;

    edges.push_back( std::make_pair<>(a,b) );

  }
  edges.push_back( std::make_pair<>((convex_hull.end()-1)->vertex_,convex_hull.begin()->vertex_) );

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

bool MISTool::GetIntersectionPolygons(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  std::vector< std::vector<SimplePoint<> > > clasper_top_polygons = GetClasperPolygons(pose);
  
  std::vector<cv::Vec3f> intersections;
  for(auto polygon = clasper_top_polygons.begin(); polygon != clasper_top_polygons.end(); polygon++ ){

    cv::Vec3f intersection;
    if(!GetIntersectionPlane(*polygon,ray,intersection)) continue;
    
    if(IntersectionInConvexHull(*polygon,intersection))
      intersections.push_back(intersection);
    
  }

  if(!intersections.size()) return false;

  //sort intersections by distance 
  double largest = 0, smallest = std::numeric_limits<double>::max();
  for(auto intersection = intersections.begin(); intersection != intersections.end(); intersection++){

    double distance = norm2(*intersection);
    if(distance < smallest){
      smallest = distance;
      front = *intersection;
    }else if(distance > largest){
      largest = distance;
      back = *intersection;
    }
  }

  return true;

}

bool MISTool::GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  cv::Vec3f tip_front,shaft_front,tip_back,shaft_back;
  bool tip = GetIntersectionTip(ray,tip_front,tip_back,pose);
  bool shaft = GetIntersectionShaft(ray,shaft_front,shaft_back,pose);
 
  if(shaft && !tip){
    front = shaft_front;
    back = shaft_back;
    return true;
  }
  if(tip && !shaft){
    front = tip_front;
    back = tip_back;
    return true;
  }
  if(!tip && !shaft){
    cv::Vec3f polygons_front,polygons_back;
    bool polygons = GetIntersectionPolygons(ray,polygons_front,polygons_back,pose);
    if(polygons) {
      front = polygons_front;
      back = polygons_back;
      return true;
    }else{
      return false;
    }
  }

  float mag_tip = (tip_front[0]*tip_front[0]) + (tip_front[1]*tip_front[1]) + (tip_front[2]*tip_front[2]);
  float mag_shaft = (shaft_front[0]*shaft_front[0]) + (shaft_front[1]*shaft_front[1]) + (shaft_front[2]*shaft_front[2]);

  if(mag_tip < mag_shaft){
    front = tip_front;
    back = tip_back;
    return true;
  }else{
    front = shaft_front;
    back = shaft_back;
    return true;
  }
    

}


bool MISTool::GetIntersectionTip(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  //cv::Mat top,bottom;
  cv::Vec3f top((float)-height_/2 + (float)height_curve_,-(radius_-radius_tip_),0.0);
  top = pose.Transform(top);
  cv::Vec3f bottom((float)-height_/2,-(radius_ - radius_tip_),0.0);
  bottom = pose.Transform(bottom);

  cv::Vec3f dP = top - bottom;
  float dPdP = dP.dot(dP);
  if(!dPdP) dPdP = 0.0000000001f;
  cv::Vec3f t_1 = ray - ((ray.dot(dP))/dPdP)*dP;
  cv::Vec3f t_2 = ((bottom.dot(dP))/dPdP)*dP - bottom;

  float a = t_1.dot(t_1);
  float b = 2*t_1.dot(t_2);
  float c = t_2.dot(t_2) - radius_tip_*radius_tip_;

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
    front_intersection = CircleIntersection(bottom,dP,ray,radius_tip_,front);
    if(!front_intersection)
      front_intersection = CircleIntersection(top,dP,ray,radius_tip_,front);
  }
  if(!front_intersection) return false;

  //check the intersection with the back of the cylinder
  bool back_intersection = false;
  if(alp_2 >= 0.0 && alp_2 <= 1.0){
    back = s2*ray;
    back_intersection = true;
  }else{
    back_intersection = CircleIntersection(bottom,dP,ray,radius_tip_,back);
    if(!back_intersection)
      back_intersection = CircleIntersection(bottom,dP,ray,radius_tip_,back);
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


bool MISTool::GetIntersectionShaft(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

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
