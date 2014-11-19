#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cinder/gl/gl.h>

#include <cinder/app/App.h>

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

void Model::Render(bool bind_texture){

  ci::gl::pushModelView();

  ci::gl::multModelView(world_to_model_coordinates_);
  
  model_->Render(bind_texture);

  ci::gl::popModelView();

}

void Model::UpdatePose(std::vector<float> &updates){
  
  std::vector<float> updates_to_base(updates.begin(), updates.begin() + world_to_model_coordinates_.GetNumDofs());
  world_to_model_coordinates_.UpdatePose(updates_to_base);

  //if these are the same, we have no articulated components to track
  if (updates.size() == world_to_model_coordinates_.GetNumDofs())
    return;

  std::vector<float> updates_to_end(updates.begin() + world_to_model_coordinates_.GetNumDofs(), updates.end());
  model_->UpdatePose(updates_to_end.begin());

}

void Model::SetPose(std::vector<float> &pose){

  world_to_model_coordinates_.SetPose(std::vector<float>(pose.begin(), pose.begin() + world_to_model_coordinates_.GetNumDofs()));
  
  std::vector<float> pose_to_end(pose.begin() + world_to_model_coordinates_.GetNumDofs(), pose.end());
  model_->SetPose(pose_to_end.begin());


}

void Model::GetPose(std::vector<float> &pose){

  pose = world_to_model_coordinates_.GetPose();
  model_->GetPose(pose);

}

std::vector<ci::Vec3f> Model::ComputeJacobian(const ci::Vec3f &point, const int target_frame_idx) const {

  //compute the jacobian for the base pose
  std::vector<ci::Vec3f> r = world_to_model_coordinates_.ComputeJacobian(point);

  //pass this vector into the articualted nodes and get their jacobians too
  
  for (int i = 0; i < 6; ++i){
    r.push_back(ci::Vec3f(0.0f, 0.0f, 0.0f));
  }
  model_->ComputeJacobianForPoint(world_to_model_coordinates_, point, target_frame_idx, r);

  assert(r[7] == ci::Vec3f(0.0f, 0.0f, 0.0f));
  assert(r[10] == ci::Vec3f(0.0f, 0.0f, 0.0f));

  r.erase(r.begin() + 7);
  r.erase(r.begin() + 9);   

  return r;

}

