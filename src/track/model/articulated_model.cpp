#include <cinder/ObjLoader.h>
#include <cinder/Json.h>
#include <cinder/ImageIo.h>
#include <cinder/app/App.h>
#include <cinder/gl/Texture.h>

#include "../../../include/track/model/articulated_model.hpp"


using namespace ttrk;

inline void LoadMeshAndTexture(boost::shared_ptr<ci::TriMesh> &mesh, boost::shared_ptr<ci::gl::Texture> &texture, ci::JsonTree &tree, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doens't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()));
  mesh.reset(new ci::TriMesh);
  loader.load(mesh.get());

  //boost::filesystem::path tex_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["texture"].getValue<std::string>());
  //if(!boost::filesystem::exists(file)) throw(std::runtime_error("Error, the file doens't exist!\n"));

  ci::gl::Texture::Format format;  
  format.enableMipmapping(true);

}

void ArticulatedNode::LoadData(ci::JsonTree &tree, ArticulatedNode::Ptr parent, const std::string &root_dir) {

  LoadMeshAndTexture(model_,texture_,tree,root_dir);
  transform_.setToIdentity();
  articulation_.setToIdentity();
  parent_ = parent;

  movable_ = tree["articulated"].getValue<bool>();

  if( !movable_ ) return;

  center_of_mass_[0] = tree["center"]["x"].getValue<double>() ;
  center_of_mass_[1] = tree["center"]["y"].getValue<double>() ;
  center_of_mass_[2] = tree["center"]["z"].getValue<double>() ;

  transform_.translate(center_of_mass_);

  axis_of_rotation_[0] = tree["axis"]["x"].getValue<double>() ;
  axis_of_rotation_[1] = tree["axis"]["y"].getValue<double>() ;
  axis_of_rotation_[2] = tree["axis"]["z"].getValue<double>() ;

  max_angle_ = tree["max_angle"].getValue<double>();
  min_angle_ = tree["min_angle"].getValue<double>();
  
 }

ci::Matrix44d ArticulatedNode::GetTransform() {
  
  if( parent_ != 0x0 ){
    return parent_->GetTransform() * transform_ * articulation_ ;
  }else{
    return transform_ * articulation_  ; //root node returns this
  }

}

void ArticulatedTool::ParseJsonTree(ci::JsonTree &jt, ArticulatedNode::Ptr node, ArticulatedNode::Ptr parent, const std::string &root_dir){

  node->LoadData(jt,parent,root_dir);
 
  //node->children_.back()->LoadData(jt,node,root_dir);
    
  for(auto child = jt.begin(); child != jt.end(); ++child){
    if(child->getKey().find("child") != std::string::npos){
      node->children_.push_back( ArticulatedNode::Ptr(new ArticulatedNode) );
      ParseJsonTree(*child,node->children_.back(), node, root_dir); //
    }
  }

}

void ArticulatedTool::LoadFromJsonFile(const std::string &json_file){

  try{

    ci::JsonTree loader(ci::loadFile(json_file));

    ParseJsonTree(loader.getChild("root"),articulated_model_->RootNode(),ArticulatedNode::Ptr(),boost::filesystem::path(json_file).parent_path().string());

  }catch(ci::Exception &e){

    if( ! boost::filesystem::exists(json_file) )
      std::cout << "Error, cannot find file : " << json_file << std::endl;

    std::cout << e.what() << "\n";
    std::cout.flush();

  }

}

ArticulatedTool::ArticulatedTool(const std::string &model_parameter_file) : articulated_model_(new ArticulatedTree) {

  const std::string ext = boost::filesystem::path(model_parameter_file).extension().string();

  if( ext == ".json" )
    LoadFromJsonFile(model_parameter_file);
  else
    throw(std::runtime_error("Error, unsupported file type.\n"));

}

void IntuitiveSurgicalLND::RotateHead(const double angle) {

  body_->Rotate(angle);

}

void IntuitiveSurgicalLND::RotateClaspers(const double angle_1,const double angle_2) {

  clasper_left_->Rotate(angle_1);
  clasper_right_->Rotate(angle_2);

}
