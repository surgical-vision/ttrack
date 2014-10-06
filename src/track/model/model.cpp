#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "../../../include/track/model/model.hpp"
#include "../../../include/utils/helpers.hpp"
#include "../../../include/utils/primitives.hpp"

using namespace ttrk;

Model::Model(const std::string &model_parameter_file){
/*
  boost::filesystem::path file(model_parameter_file);
  if(boost::filesystem::extension(file) == ".obj")
    ReadFromObjFile(model_parameter_file);
  else
    throw(std::runtime_error("Error, unsupported file type for mode: " + model_parameter_file));
  
  */
}

void Model::InitGL(){

  if (model_->getVertices().size() == 0){
    throw std::runtime_error("Error, model is not loaded");
  }



}


void Model::Render(){

  auto meshes_textures_transforms = GetRenderableMeshes();

  for (auto mesh_tex_trans = meshes_textures_transforms.begin(); mesh_tex_trans != meshes_textures_transforms.end(); ++mesh_tex_trans){

    auto texture = mesh_tex_trans->get<1>();

    ci::gl::pushModelView();
    ci::gl::multModelView(mesh_tex_trans->get<2>());

    const auto trimesh = mesh_tex_trans->get<0>();
    ci::gl::draw(*trimesh);

    ci::gl::popModelView();


  }

}