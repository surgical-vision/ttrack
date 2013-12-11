#include "../../../headers/track/model/model.hpp"
#include "../../../headers/utils/helpers.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "../../../headers/utils/primitives.hpp"

using namespace ttrk;


bool Model::GetIntersectionTriangle(const cv::Vec3f &ray, const cv::Vec3f &vert0, const cv::Vec3f &vert1, const cv::Vec3f &vert2, float &distance) const {

  cv::Vec3f edge1, edge2, tvec, pvec, qvec;
  float det;
  float u, v;
  const float EPSILON = 0.000001f;

  edge1 = vert1 - vert0;
  edge2 = vert2 - vert0;

  pvec = ray.cross( edge2 );
  det = edge1.dot( pvec );

  if( det > -EPSILON && det < EPSILON )
    return false;

  float inv_det = 1.0f / det;
  tvec = cv::Vec3f(0,0,0) - vert0;
  u = tvec.dot( pvec ) * inv_det;
  if( u < 0.0f || u > 1.0f )
    return false;

  qvec = tvec.cross( edge1 );

  v = ray.dot( qvec ) * inv_det;
  if( v < 0.0f || u + v > 1.0f )
    return 0;

  distance = edge2.dot( qvec ) * inv_det;
  return true;

}

bool Model::GetIntersectionQuadrilateral(const cv::Vec3f &ray, const cv::Vec3f &vert0, const cv::Vec3f &vert1, const cv::Vec3f &vert2, const cv::Vec3f &vert3, float &distance) const {
  return false;
}

MISTool::MISTool(const std::string &model_parameter_file){

  std::vector<cv::Vec3f> vertices; 
  std::vector<cv::Vec3i> mesh_indices;

  std::ifstream file(model_parameter_file);
  if( !file.is_open() ) throw(std::runtime_error("Error, could not open file!\n"));

  for(;;){


    char line_type;
    file >> line_type;

    if(file.eof()) break;

    if(line_type == 'v'){

      char isnormal = file.peek();

      if(isnormal == 'n') {
        std::getline(file,std::string());
      }
      else {
        vertices.push_back(ReadVertexFromObj(file));
      }
    }
    else if(line_type == 'f'){
      mesh_indices.push_back(ReadFaceFromObj(file));
    }
    else{
      std::getline(file,std::string());
    }


  }


  flattened_triangles_.reserve( 3*3*mesh_indices.size() );


  for(auto triangle = mesh_indices.begin(); triangle != mesh_indices.end(); ++triangle){

    for(int i=0;i<3;++i){

      const cv::Vec3f &v = vertices[(*triangle)[i]];

      for(int j=0;j<3;j++) flattened_triangles_.push_back(v[j]);

    }


  }



}


cv::Vec3f Model::ReadVertexFromObj(std::ifstream &ifs) const {

  cv::Vec3f f;
  ifs >> f[0] >> f[1] >> f[2];
  std::getline(ifs,std::string());
  return f;

}


cv::Vec3i Model::ReadFaceFromObj(std::ifstream &ifs) const {

  //char space;
  //ifs >> space; //skip the space at the start

  std::string str;
  std::getline(ifs,str);


  //check for 
  std::vector<std::string> strs;
  boost::split(strs, str, boost::is_any_of(" "));

  if(strs.front() == "") strs.erase(strs.begin());

  if(strs.size() != 3) throw(std::runtime_error("Error reading face from obj: " + str + "\n"));

  cv::Vec3i ret;

  for(size_t i = 0; i < strs.size(); ++i){

    std::vector<std::string> vals;
    boost::split(vals, strs[i], boost::is_any_of("/"));
    std::stringstream ss(vals[0]);
    ss >> ret[i];

  }

  std::getline(ifs,std::string()); //move to the end fo the line
  ret = cv::Vec3i(ret[0]-1,ret[1]-1,ret[2]-1); //obj file uses 1...N indexing rather than 0...N-1 indexing

  return ret;

}


cv::Vec3f MISTool::GetTrackedPoint() const {
  throw(std::runtime_error("Error, do this!\n"));
  return cv::Vec3f (-(float)height_/2 + (float)(height_tip_), 
    (float) - 0.15*radius_*2,
    0);

}

boost::shared_ptr<std::vector<Object *> > Model::GetPoints(const Pose &pose) {

  static bool first = true;

  if(first){
    objects_.reset(new std::vector<Object *>());
    objects_->reserve(flattened_triangles_.size()/3);
  }

  std::cerr << cv::Point3f(pose.translation_) << "\n";
  std::cerr << pose.rotation_ << "\n";

  size_t i=0;
  for(auto triangle = flattened_triangles_.begin(); triangle != flattened_triangles_.end(); triangle = triangle + 9){

    cv::Vec3f vertices[3];
    for(int i=0;i<3;++i){
      const int index = 3*i;
      vertices[i] = pose.Transform( cv::Vec3f( triangle[index],triangle[index+1],triangle[index+2] ) );
    }

    if(first){
      objects_->push_back( 
        new Triangle( 
        Vector3(vertices[0][0],vertices[0][1],vertices[0][2]), 
        Vector3(vertices[1][0],vertices[1][1],vertices[1][2]),
        Vector3(vertices[2][0],vertices[2][1],vertices[2][2]) ) 
        );
    }else{
      *(objects_->operator[](i)) =  Triangle( 
        Vector3(vertices[0][0],vertices[0][1],vertices[0][2]), 
        Vector3(vertices[1][0],vertices[1][1],vertices[1][2]),
        Vector3(vertices[2][0],vertices[2][1],vertices[2][2]) 
        );
    }

  }
  first = false;
  bvh_.reset(new BVH(objects_.get())); //as MIS tool destructs AFTER bvh, this should be safe
  return objects_;
}


std::vector<SimplePoint<> > MISTool::Points(const Pose &pose) const {

  return std::vector<SimplePoint<> >();
}


bool Model::ComputeAllIntersections(boost::shared_ptr<MonocularCamera> cam, const Pose &pose, const int rows, const int cols, cv::Mat &intersection_image) const{
  //bool Model::GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const {

  //when loading the model find the largest +ve x,y,z  and the largest -ve x,y,z
  //compute bounding box and check intersection with that

  std::vector<Quadrilateral> bounding_box;// = ComputeBoundingBox(pose);


  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){

      cv::Vec3f ray = cam->UnProjectPoint(cv::Point(c,r));

      bool intersects = false;
      for(auto quad = bounding_box.begin(); quad != bounding_box.end(); quad++){

        //intersects = GetIntersectionQuadrilateral(ray, quad->vertices[0], quad->vertices[1], quad->vertices[2], quad->vertices[3], cv::Vec3f(), cv::Vec3f(), double() );

      }

      if(!intersects) return false;
    }
  }
  //create local vector of points and tranform it transform points - we can ignore the neighbors etc

  //for triangle in mesh
  //check intersection with triangle


}

/*std::vector<Quadrilateral> MISTool::ComputeBoundingBox(const Pose &pose) const {



}
*/

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

  std::vector< std::vector<SimplePoint<> > > clasper_top_polygons;// = GetClasperPolygons(pose);
  throw(std::runtime_error("Err"));

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
    }
    if(distance > largest){
      largest = distance;
      back = *intersection;
    }
  }

  return true;

}
