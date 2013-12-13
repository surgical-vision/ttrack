#include "../../../headers/track/model/model.hpp"
#include "../../../headers/utils/helpers.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "../../../headers/utils/primitives.hpp"
#include <boost/filesystem.hpp>

using namespace ttrk;

Model::Model(const std::string &model_parameter_file){

  boost::filesystem::path file(model_parameter_file);
  if(boost::filesystem::extension(file) == ".obj")
    ReadFromObjFile(model_parameter_file);
  else
    throw(std::runtime_error("Error, unsupported file type for mode: " + model_parameter_file));

}

void Model::ReadFromObjFile(const std::string &model_parameter_file){
  
  std::vector<cv::Vec3d> vertices; 
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

      const cv::Vec3d &v = vertices[(*triangle)[i]];

      for(int j=0;j<3;j++) flattened_triangles_.push_back(v[j]);

    }

  }

  tracked_point_ = cv::Vec3d(0,0,0);



}

cv::Vec3d Model::ReadVertexFromObj(std::ifstream &ifs) const {

  cv::Vec3d f;
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
    ss >> ret[(int)i];

  }

  ret = cv::Vec3i(ret[0]-1,ret[1]-1,ret[2]-1); //obj file uses 1...N indexing rather than 0...N-1 indexing

  return ret;

}

bool Model::GetIntersection(const cv::Vec3d &ray, cv::Vec3d &front, cv::Vec3d &back) const{
  
  IntersectionInfo I;
  //NOTE needs to be updated so that the back intersection is estimated too
  bvh_->getIntersection( Ray( Vector3(0,0,0),Vector3(ray[0],ray[1],ray[2]) ), &I, false);
  
  if(I.object != 0x0){
    front = cv::Vec3d(I.hit[0],I.hit[1],I.hit[2]);
    back = cv::Vec3d(I.hit[0],I.hit[1],I.hit[2]);
    return true;
  }else{
    return false;
  }
  
}

void Model::UpdateMeshCache(const Pose &pose) {

  static bool first = true;

  cached_mesh_.pose_ = pose;

  if(first){
    cached_mesh_.mesh_.reset(new std::vector<Object *>());
    cached_mesh_.mesh_->reserve(flattened_triangles_.size()/3);
  }

  size_t index = 0;
  for(auto triangle = flattened_triangles_.begin(); triangle != flattened_triangles_.end(); triangle = triangle + 9,++index){

    cv::Vec3d vertices[3];
    for(int i=0;i<3;++i){
      const int index = 3*i;
      vertices[i] = pose.Transform( cv::Vec3d( triangle[index],triangle[index+1],triangle[index+2] ) );
    }

    if(first){
      cached_mesh_.mesh_->push_back( 
        new Triangle( 
        Vector3(vertices[0][0],vertices[0][1],vertices[0][2]), 
        Vector3(vertices[1][0],vertices[1][1],vertices[1][2]),
        Vector3(vertices[2][0],vertices[2][1],vertices[2][2]) ) 
        );
    }else{
      *(cached_mesh_.mesh_->operator[](index)) =  Triangle( 
        Vector3(vertices[0][0],vertices[0][1],vertices[0][2]), 
        Vector3(vertices[1][0],vertices[1][1],vertices[1][2]),
        Vector3(vertices[2][0],vertices[2][1],vertices[2][2]) 
        );
    }

  }

  first = false;
  bvh_.reset(new BVH(cached_mesh_.mesh_.get())); //as MIS tool destructs AFTER bvh, this should be safe

}



boost::shared_ptr<std::vector<Object *> > Model::Points(const Pose &pose) {

  if(cached_mesh_.pose_ != pose) UpdateMeshCache(pose);
  return cached_mesh_.mesh_;
  
}


