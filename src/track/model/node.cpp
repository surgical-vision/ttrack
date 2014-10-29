#include <cinder/ObjLoader.h>
#include <cinder/gl/Texture.h>
#include <cinder/app/App.h>
#include <cinder/gl/Light.h>

#include "../../../include/track/model/node.hpp"
#include "../../../include/track/model/dh_helpers.hpp"


using namespace ttrk;

void Node::LoadMeshAndTexture(ci::JsonTree &tree, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()));
  loader.load(&model_, true, false, false);
  vbo_ = ci::gl::VboMesh(model_);

}

void Node::Render(){
  
  ci::gl::pushModelView();

  auto x = GetRelativeTransform();

  ci::app::console() << "Rendering at transform = \n" << x << std::endl;

  ci::gl::multModelView(GetRelativeTransform());

  glEnable(GL_COLOR_MATERIAL);

  if (model_.getNumVertices() != 0)
    ci::gl::draw(vbo_);

  glDisable(GL_COLOR_MATERIAL);

  ci::gl::popModelView();

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->Render();
  }

}

Node::Ptr Node::GetChildByIdx(std::size_t &curr_idx, const std::size_t target_idx){

  if (curr_idx == target_idx) return boost::shared_ptr<Node>(this);

  for (size_t i = 0; i < children_.size(); ++i){
    Node::Ptr c = children_[i]->GetChildByIdx(curr_idx, target_idx);
    if (c != 0x0) return c;
    curr_idx++;
  }

}


ci::Matrix44f DHNode::GetWorldTransform(const ci::Matrix44f &base_frame_transform) const {
  
  if (parent_ != 0x0){
    DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
    //return p->ComputeDHTransform() * ComputeDHTransform();
    return glhMultMatrixRight(p->GetWorldTransform(base_frame_transform), ComputeDHTransform());
  }
  else{ 
    return base_frame_transform;
  }

}

ci::Matrix44f DHNode::GetRelativeTransform() const {

  if (parent_ != 0x0){
    DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
    //return p->ComputeDHTransform() * ComputeDHTransform();
    return glhMultMatrixRight(p->GetRelativeTransform(), ComputeDHTransform());
  }
  else{
    ci::Matrix44f m;
    m.setToIdentity();
    return m;// ComputeDHTransform();
  }

}

void DHNode::LoadData(ci::JsonTree &tree, Node::Ptr parent, const std::string &root_dir){

  if (parent != 0x0){
    parent_ = boost::dynamic_pointer_cast<DHNode>(parent);
  }
  else{
    parent_ = 0x0;
  }

  try{
    LoadMeshAndTexture(tree, root_dir);
  }
  catch (ci::JsonTree::Exception &){

  }

  update_ = 0.0;

  try{
    
    alpha_ = tree["dh"]["alpha"].getValue<double>();
    theta_ = tree["dh"]["theta"].getValue<double>();
    a_ = tree["dh"]["a"].getValue<double>();
    d_ = tree["dh"]["d"].getValue<double>();
    
    if (tree["dh"]["type"].getValue<std::string>() == "rotation")
      type_ = Rotation;
    else if (tree["dh"]["type"].getValue<std::string>() == "translation")
      type_ = Translation;
    else if (tree["dh"]["type"].getValue<std::string>() == "fixed")
      type_ = Fixed;
    else
      throw std::runtime_error("Error, bad value in JSON file.");
  }
  catch (ci::JsonTree::Exception &){

    //should just give the identity transform for this node (useful e.g. for the root node).
    alpha_ = M_PI / 2;
    theta_ = M_PI / 2;
    a_ = 0.0;
    d_ = 0.0;
    type_ = Fixed;

  }

  ci::JsonTree children = tree.getChild("children");
  for (int i = 0; i < children.getChildren().size(); ++i){
    Node::Ptr n(new DHNode);
    n->LoadData(children[i], DHNode::Ptr(this), root_dir);
    AddChild(n);
  }

}


ci::Matrix44f DHNode::ComputeDHTransform() const {
  
  ci::Matrix44f DH;
  DH.setToIdentity();
  
  if (type_ == Rotation)
    glhDenavitHartenberg(a_, alpha_, d_ + update_, theta_, DH.m);
  else if (type_ == Translation)
    glhDenavitHartenberg(a_, alpha_, d_, theta_ + update_, DH.m);
  else
    glhDenavitHartenberg(a_, alpha_, d_, theta_, DH.m);

  return DH;

}

/*

extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[4], A, psm.jnt_pos[4]);
wrist_pitch = ci::Matrix44d(A);

//don't actually care about wrist yaw as the clasper pose 'contains' this information
extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[5], A, psm.jnt_pos[5]);
ci::Matrix44d wrist_yaw = ci::Matrix44d(A);

//transform into the clasper reference frame
extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[6], A, 0);  //no dh param here as this just points in the direction of the instrument head

//rotate the instrument claspers around the clasper axis
grip1 = ci::Matrix44d(A);
grip2 = ci::Matrix44d(A);

//this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
double val = psm.jnt_pos[6];
grip1.rotate(ci::Vec3d(0, 1, 0), 0.5*val);

grip2.rotate(ci::Vec3d(0, 0, 1), M_PI);
grip2.rotate(ci::Vec3d(0, 1, 0), 0.5*val);

*/