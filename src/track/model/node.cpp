#include <cinder/ObjLoader.h>
#include <cinder/gl/Texture.h>
#include <cinder/app/App.h>
#include <cinder/gl/Light.h>

#include "../../../include/track/model/node.hpp"
#include "../../../include/track/model/dh_helpers.hpp"


using namespace ttrk;

Node::~Node(){ }

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
  
  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelView(GetRelativeTransform());

    glEnable(GL_COLOR_MATERIAL);

    if (model_.getNumVertices() != 0)
      ci::gl::draw(vbo_);

    glDisable(GL_COLOR_MATERIAL);

    ci::gl::popModelView();
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->Render();
  }

}



Node::Ptr Node::GetChildByIdx(std::size_t &curr_idx, const std::size_t target_idx){

  if (curr_idx == target_idx) return boost::shared_ptr<Node>(this);
  
  curr_idx++;

  for (size_t i = 0; i < children_.size(); ++i){
    Node::Ptr c = children_[i]->GetChildByIdx(curr_idx, target_idx);
    if (c != nullptr) return c;
  }

  return Node::Ptr(nullptr);

}


void Node::ComputeJacobianForPoint(const ci::Vec3f &point, std::vector<ci::Vec3f> &jacobian){

  //the node with null parent is the root node so it's pose is basically just the base pose.
  if (parent_ != nullptr){

    //derivative of transform k = 
    //T_1 = transform from frame which point resides to parent of this frame (i.e. closest frame to point)
    //T_2 = transform from parent of this frame to this frame
    //T_3 = transfrom from this frame to origin - with GetRelativeTransform
    


    jacobian.push_back(ci::Vec3f(0.0f, 0.0f, 0.0f)); //just for testing
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->ComputeJacobianForPoint(point, jacobian);
  }

}

void DHNode::UpdatePose(std::vector<float>::iterator &updates){
 
  //the node with null parent is the root node so it's pose is basically just the base pose and therefore there is not an update
  if (parent_ != nullptr){
    update_ = *updates;
    ++updates;
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->UpdatePose(updates);
  }

}

ci::Matrix44f DHNode::GetWorldTransform(const ci::Matrix44f &base_frame_transform) const {
  
  if (parent_ != 0x0){
    DHNode *p = dynamic_cast<DHNode *>(parent_);
    //return p->ComputeDHTransform() * ComputeDHTransform();
    return glhMultMatrixRight(p->GetWorldTransform(base_frame_transform), ComputeDHTransform());
  }
  else{ 
    return base_frame_transform;
  }

}

ci::Matrix44f DHNode::GetRelativeTransform() const {

  if (parent_ != 0x0){
    DHNode *p = dynamic_cast<DHNode *>(parent_);
    //return p->ComputeDHTransform() * ComputeDHTransform();
    return glhMultMatrixRight(ComputeDHTransform(), p->GetRelativeTransform());
  }
  else{
    ci::Matrix44f m;
    m.setToIdentity();
    return m;// ComputeDHTransform();
  }

}

void DHNode::LoadData(ci::JsonTree &tree, Node *parent, const std::string &root_dir){

  if (parent != 0x0){
    parent_ = dynamic_cast<DHNode *>(parent);
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
    
    alpha_ = tree["dh"]["alpha"].getValue<float>();
    theta_ = tree["dh"]["theta"].getValue<float>();
    a_ = tree["dh"]["a"].getValue<float>();
    d_ = tree["dh"]["d"].getValue<float>();
    
    if (tree["dh"]["type"].getValue<std::string>() == "rotation")
      type_ = Rotation;
    else if (tree["dh"]["type"].getValue<std::string>() == "translation")
      type_ = Translation;
    else if (tree["dh"]["type"].getValue<std::string>() == "fixed")
      type_ = Fixed;
    else
      throw std::runtime_error("Error, bad value in JSON file.");

    alt_axis_ = ci::Vec3f(0, 0, 0);

  }
  catch (ci::JsonTree::Exception &){

    try{

      ci::JsonTree rotate_axis = tree.getChild("rotate");
      alt_axis_[0] = rotate_axis[0].getValue<float>();
      alt_axis_[1] = rotate_axis[1].getValue<float>();
      alt_axis_[2] = rotate_axis[2].getValue<float>();
      type_ = Alternative;

    }
    catch (ci::JsonTree::Exception &){

      type_ = Fixed;

    }

    //should just give the identity transform for this node (useful e.g. for the root node).
    alpha_ = (float)M_PI / 2;
    theta_ = (float)M_PI / 2;
    a_ = 0.0f;
    d_ = 0.0f;
    

  }

  ci::JsonTree children = tree.getChild("children");
  for (size_t i = 0; i < children.getChildren().size(); ++i){
    Node::Ptr n(new DHNode);
    n->LoadData(children[i], this, root_dir);
    AddChild(n);
  }

}

void DHNode::createFixedTransform(const ci::Vec3f &axis, const float rads, ci::Matrix44f &output) const {

  output = ci::Matrix44f::createRotation(axis, rads);

}


ci::Matrix44f DHNode::GetTransformBetweenNodes(const Node *from, const Node *to){

  //reached the last coordinate system in the chain.
  if (from == this){
    return ComputeDHTransform();
  }
  if (parent_ != 0x0){
    DHNode *p = dynamic_cast<DHNode *>(parent_);

    return glhMultMatrixRight(ComputeDHTransform(), p->GetRelativeTransform());
  }
  else{
    ci::Matrix44f m;
    m.setToIdentity();
    return m;// ComputeDHTransform();
  }



}


ci::Matrix44f DHNode::ComputeDHTransform() const {
  
  ci::Matrix44f DH;
  DH.setToIdentity();
  
  if (type_ == Translation)
    glhDenavitHartenberg(a_, alpha_, d_ + update_, theta_, DH.m);
  else if (type_ == Rotation)
    glhDenavitHartenberg(a_, alpha_, d_, theta_ + update_, DH.m);
  else if (type_ == Fixed)
    glhDenavitHartenberg(a_, alpha_, d_, theta_, DH.m);
  else if (type_ == Alternative)
    createFixedTransform(alt_axis_, update_, DH);

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