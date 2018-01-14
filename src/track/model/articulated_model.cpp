#include <cinder/ObjLoader.h>
#include <cinder/gl/gl.h>
#include <cinder/Json.h>
#include <cinder/ImageIo.h>
#include <cinder/app/App.h>
#include <cinder/gl/Texture.h>

#include "../../../include/ttrack/track/model/articulated_model.hpp"


using namespace ttrk;

DenavitHartenbergArticulatedModel::DenavitHartenbergArticulatedModel(const std::string &model_parameter_file, const std::string &save_file) {

  save_file_ = save_file;

  LoadFromFile(model_parameter_file);

  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(7.63, 2.81, -0.90), "Logo Dot 1", 1));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(9.27, 2.82, -0.08), "Front Pin 1", 1));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(5.90, 3.00, -0.034), "Logo 1", 1));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(3.27, -1.47, 2.90), "Wheel Pin 1", 1));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(3.18, 2.97, -0.01), "Rear Pin 1", 1));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(-0.584, 3.307, 0.01), "Shaft Pin 1", 0));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(2.79, 0.39, 9.24), "Clasper 1", 4));
  icl_data.push_back(ICL_Tracked_Point(ci::Vec3f(-2.45, 0.122, 9.3), "Clasper 2", 5));

}

void DenavitHartenbergArticulatedModel::ParseJson(ci::JsonTree &jt, const std::string &root_dir){

  model_.reset(new DHNode); 
  size_t start_idx = 0;
  model_->LoadData(jt, 0x0, root_dir, start_idx);

}
