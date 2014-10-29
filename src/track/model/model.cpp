#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cinder/gl/gl.h>

#include "../../../include/track/model/model.hpp"
#include "../../../include/utils/helpers.hpp"

using namespace ttrk;

Model::Model(const std::string &model_parameter_file){

  LoadFromFile(model_parameter_file);

}


Model::Model(Node::Ptr component, const ci::Matrix44f &world_to_model_transform){

  model_ = component;
  world_to_model_coordinates_ = Pose(component->GetWorldTransform(world_to_model_transform)); //need to write the get world transform bit

}

void Model::LoadFromFile(const std::string &filename){

  boost::filesystem::path p(filename);
  if (p.extension().string() == ".json"){
    LoadJson(filename);
  }
  else{
    throw std::runtime_error("Error, unsupported file type");
  }

}

void Model::LoadJson(const std::string &filename){

  try{

    ci::JsonTree loader(ci::loadFile(filename));

    ParseJson(loader.getChild("root"), boost::filesystem::path(filename).parent_path().string());

  }
  catch (ci::Exception &e){

    if (!boost::filesystem::exists(filename))
      std::cout << "Error, cannot find file : " << filename << std::endl;

    std::cout << e.what() << "\n";
    std::cout.flush();

  }

}


void Model::Render(){

  ci::gl::pushModelView();
  ci::gl::multModelView(world_to_model_coordinates_);

  model_->Render();

  ci::gl::popModelView();

}