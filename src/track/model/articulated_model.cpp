#include <cinder/ObjLoader.h>
#include <cinder/gl/gl.h>
#include <cinder/Json.h>
#include <cinder/ImageIo.h>
#include <cinder/app/App.h>
#include <cinder/gl/Texture.h>

#include "../../../include/track/model/articulated_model.hpp"


using namespace ttrk;

DenavitHartenbergArticulatedModel::DenavitHartenbergArticulatedModel(const std::string &model_parameter_file) { 
  
  LoadFromFile(model_parameter_file);

}

void DenavitHartenbergArticulatedModel::ParseJson(ci::JsonTree &jt, const std::string &root_dir){

  model_.reset(new DHNode); 
  size_t start_idx = 0;
  model_->LoadData(jt, 0x0, root_dir, start_idx);

}


boost::shared_ptr<Model> DenavitHartenbergArticulatedModel::GetComponent(const std::size_t component_idx){

  return boost::shared_ptr<Model>(new DenavitHartenbergArticulatedModel(model_->GetChildByIdx(component_idx), world_to_model_coordinates_));

}