#include <cinder/ObjLoader.h>
#include <cinder/gl/Texture.h>
#include <cinder/app/App.h>
#include <cinder/gl/Light.h>
#include <cinder/ImageIo.h>

#include "../../../include/ttrack/track/model/node.hpp"
#include "../../../include/ttrack/track/model/dh_helpers.hpp"

using namespace ttrk;

Node::~Node(){ }

void Node::LoadMeshAndTexture(ci::JsonTree &tree, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path texture_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["tex-file"].getValue<std::string>());
  if (!boost::filesystem::exists(texture_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()), true);
  loader.load(&model_, true, true, false);

  texture_ = ci::gl::Texture( ci::loadImage( texture_file.string()));

  vbo_ = ci::gl::VboMesh(model_);

}


void Node::RenderMaterial(){

  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelView(GetRelativeTransformToRoot());

    glEnable(GL_COLOR_MATERIAL);

    if (model_.getNumVertices() != 0)
      ci::gl::draw(vbo_);
    
    glDisable(GL_COLOR_MATERIAL);
    
    ci::gl::popModelView();
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->RenderMaterial();
  }

}

void Node::RenderTexture(int id){
  
  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelView(GetRelativeTransformToRoot()); 

    if (texture_){
      if (id == 0)
        texture_.enableAndBind();
      else
        texture_.bind(id);

    }

    if (model_.getNumVertices() != 0)
      ci::gl::draw(vbo_);

    if (texture_){
      texture_.disable();
      texture_.unbind(id);
    }
    else{
      glDisable(GL_COLOR_MATERIAL);
    }


    ci::gl::popModelView();
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->RenderTexture(id);
  }

}

bool Node::PerformPicking(const ci::Matrix44f &mvm, const ci::Vec3f &ray, ci::Vec3f &intersection, ci::Vec3f &normal) const{

  float result = 1.0e6f;
  size_t polycount = model_.getNumTriangles();
  float distance = 0.0f;
  ci::Ray ray_(ci::Vec3f(0, 0, 0), ray);

  for (size_t i = 0; i<polycount; ++i)
  {
    ci::Vec3f v0, v1, v2;
    // get a single triangle from the mesh
    model_.getTriangleVertices(i, &v0, &v1, &v2);

    // transform triangle to world space
    v0 = mvm.transformPointAffine(v0);
    v1 = mvm.transformPointAffine(v1);
    v2 = mvm.transformPointAffine(v2);


    // test to see if the ray intersects with this triangle
    if (ray_.calcTriangleIntersection(v0, v1, v2, &distance)) {
      // set our result to this if its closer than any intersection we've had so far
      if (distance < result) {
        result = distance;
        // assuming this is the closest triangle, we'll set our normal
        // while we've got all the points handy
        normal = (v1 - v0).cross(v2 - v0).normalized();
      }
    }
  }

  // did we have a hit?
  if (distance > 0) {
    intersection = ray_.calcPosition(result);
    return true;
  }
  else
    return false;

  for (size_t i = 0; i < children_.size(); ++i){
    bool x = children_[i]->PerformPicking(mvm, ray, intersection, normal);
    if (x) return x;
  }

}

std::vector<Node::ConstPtr> Node::GetChildren() const {

  std::vector<Node::ConstPtr> ret;
  std::copy(children_.cbegin(), children_.cend(), ret.begin());
  return ret;

}

Node *Node::GetChildByIdx(const std::size_t target_idx){

  if (idx_ == target_idx) return this; //return shared from this

  for (size_t i = 0; i < children_.size(); ++i){
    Node *c = children_[i]->GetChildByIdx(target_idx);
    if (c != nullptr) return c;
  }

  return nullptr;

}

const Node *Node::GetChildByIdx(const std::size_t target_idx) const{

  if (idx_ == target_idx) return this;

  for (size_t i = 0; i < children_.size(); ++i){
    const Node *c = children_[i]->GetChildByIdx(target_idx);
    if (c != nullptr) return c;
  }

  return nullptr;

}

void Node::ComputeJacobianForPoint(const ci::Matrix44f &world_transform, const ci::Vec3f &point, const int target_frame_index, std::vector<ci::Vec3f> &jacobian) const {

  //the node with null parent is the root node so it's pose is basically just the base pose.
  if (parent_ != nullptr){

    //derivative of transform k = 
    //T_1 = transform from frame which point resides to parent of this frame (i.e. closest frame to point)
    //z = rotation axis of point
    //T_3 = transfrom from this frame to origin - with GetRelativeTransform

    if (!NodeIsChild(target_frame_index)){// || !NodeIsTransformable() ){

      jacobian.push_back(ci::Vec3f(0.0f, 0.0f, 0.0f));

    }
    else{

      ci::Matrix44f T_1 = GetRelativeTransformToChild(target_frame_index);

      //ci::Vec4f z(GetAxis(), 1);

      
      //ci::Vec4f z = GetRelativeTransformToRoot() * ci::Vec4f(GetAxis(), 1);

      ci::Matrix44f T_3 = GetRelativeTransformToRoot() * world_transform;

      ci::Vec4f z = GetTransformFromParent() * ci::Vec4f(GetAxis(), 1);
      
      ci::Vec4f end = T_3 * ci::Vec4f(point, 1);
      
      ci::Vec4f jac = T_1 * (z.cross(end));

      jacobian.push_back(ci::Vec3f(jac[0], jac[1], jac[2]));

    }

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  }

}

bool Node::NodeIsChild(const size_t child_idx) const {

  const Node *c = GetChildByIdx(child_idx);
  if (c == nullptr) return false;
  else return true;

}

bool DHNode::NodeIsTransformable() const {

  return type_ == JointType::Rotation || type_ == JointType::Translation || type_ == JointType::Alternative;

}

void DHNode::GetPose(std::vector<float> &pose) const {

  if (parent_ != nullptr && NodeIsTransformable()){
    pose.push_back(update_);
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->GetPose(pose);
  }

}

void DHNode::ComputeJacobianForPoint(const ci::Matrix44f &world_transform, const ci::Vec3f &point, const int target_frame_index, std::vector<ci::Vec3f> &jacobian) const {

  if (target_frame_index == 0){

    jacobian[7 + idx_] = ci::Vec3f(0.0f,0.0f,0.0f);

  }

  else if (target_frame_index == 1){

    ComputeJacobianForHead(world_transform, point, jacobian);

  }

  else if (target_frame_index == 4 || target_frame_index == 5){

    ComputeJacobianForClasperYaw(world_transform, point, jacobian);
    ComputeJacobianForClasperRotate(world_transform, point, target_frame_index, jacobian);

  }
  
  else{

    jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  }

}

void DHNode::ComputeJacobianForHead(const ci::Matrix44f &world_transform, const ci::Vec3f &point, std::vector<ci::Vec3f> &jacobian) const{

  if (!NodeIsChild(2)){

    assert(idx_ == 2 || idx_ == 3 || idx_ == 4 || idx_ == 5);
    jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);

  }
  else{

    if (idx_ == 1){

      //get point in the head's coordiante system
      ci::Matrix44f transform_to_joint = world_transform;
      glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);

      //get transform to the parent
      ci::Matrix44f transform_to_joint_parent = world_transform;
      glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);

      const ci::Matrix44f derivative_transform = GetDerivativeTransfromFromParent();

      const ci::Vec4f point_in_joint_coords = transform_to_joint.inverted() * point;

      const ci::Vec4f jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;

      jacobian[7 + idx_] = jac.xyz();

    }

  }

}

void DHNode::ComputeJacobianForClasperYaw(const ci::Matrix44f &world_transform, const ci::Vec3f &point, std::vector<ci::Vec3f> &jacobian) const{

  if (!NodeIsChild(3)){

    assert(idx_ == 3 || idx_ == 4 || idx_ == 5);

    jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);

  }

  if (idx_ == 1 || idx_ == 2){


    ci::Matrix44f transform_to_joint = world_transform;
    glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);

    //get transform to the parent
    ci::Matrix44f transform_to_joint_parent = world_transform;
    glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);

    const ci::Matrix44f derivative_transform = GetDerivativeTransfromFromParent();

    const ci::Vec4f point_in_joint_coords = transform_to_joint.inverted() * point;

    const ci::Vec4f jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;

    jacobian[7 + idx_] = jac.xyz();

    //ci::Vec4f jac = world_transform * parent_->GetRelativeTransformToRoot() * ci::Vec4f(dz.xyz(), 0.0f);
    //jacobian[7 + idx_] = ci::Vec3f(jac[0], jac[1], jac[2]);
      
  }

}

void DHNode::ComputeJacobianForClasperRotate(const ci::Matrix44f &world_transform, const ci::Vec3f &point, const int target_frame_index, std::vector<ci::Vec3f> &jacobian) const {

  if (target_frame_index == 5 && idx_ == 4){
    jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);
    return;
  }

  if (target_frame_index == 4 && idx_ == 5){
    jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);
    return;
  }

  if (idx_ == 4 || idx_ == 5){ //ignore 3

    ci::Matrix44f transform_to_joint = world_transform;
    glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);

    //get transform to the parent
    ci::Matrix44f transform_to_joint_parent = world_transform;
    glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);

    const ci::Matrix44f derivative_transform = GetDerivativeTransfromFromParent();

    const ci::Vec4f point_in_joint_coords = transform_to_joint.inverted() * point;

    const ci::Vec4f jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;

    jacobian[7 + idx_] = jac.xyz();

    //ci::Vec4f jac = world_transform * parent_->GetRelativeTransformToRoot() * ci::Vec4f(dz.xyz(), 0.0f);
    //jacobian[7 + idx_] = ci::Vec3f(jac[0], jac[1], jac[2]);

  }
  
}

void DHNode::SetPose(std::vector<float>::iterator &pose){

  if (parent_ != nullptr && NodeIsTransformable()){
    update_ = *pose;
    ++pose;
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->SetPose(pose);
  }


}

void DHNode::UpdatePose(std::vector<float>::iterator &updates){
 
  //the node with null parent is the root node so it's pose is basically just the base pose and therefore there is not an update
  if (parent_ != nullptr && NodeIsTransformable()){

    update_ += *updates;
    ++updates;  

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->UpdatePose(updates);
  }

}

ci::Matrix44f DHNode::GetRelativeTransformToNodeByIdx(const int target_idx) const{
  
  //for nodes that are parents of the target node just return identity
  if (target_idx >= idx_){
    return ci::Matrix44f();
  }
  //before we get to the target node, return the transform to the parent * recursive call up the chain to parent
  else{
    if (parent_ != 0x0){
      DHNode *p = dynamic_cast<DHNode *>(parent_);
      return glhMultMatrixRight(GetTransformFromParent(), p->GetRelativeTransformToNodeByIdx(target_idx));
    }
    else{
      ci::Matrix44f m;
      m.setToIdentity();
      return m;
    }
  }
  

}

ci::Matrix44f DHNode::GetRelativeTransformToChild(const int child_idx) const {

  if (child_idx == idx_)
    return ci::Matrix44f();
  else
    return GetChildByIdx(child_idx)->GetRelativeTransformToNodeByIdx(idx_);

}

ci::Matrix44f DHNode::GetWorldTransform(const ci::Matrix44f &base_frame_transform) const {
  
  if (parent_ != 0x0){
    DHNode *p = dynamic_cast<DHNode *>(parent_);
    //return p->GetTransformToParent() * GetTransformToParent();
    return glhMultMatrixRight(GetTransformFromParent(), p->GetWorldTransform(base_frame_transform));
  }
  else{ 
    return base_frame_transform;
  }

}

ci::Matrix44f DHNode::GetRelativeTransformToRoot() const {

  if (parent_ != 0x0){
    DHNode *p = dynamic_cast<DHNode *>(parent_);
    //return p->GetTransformToParent() * GetTransformToParent();
    return glhMultMatrixRight(GetTransformFromParent(), p->GetRelativeTransformToRoot());
  }
  else{
    ci::Matrix44f m;
    m.setToIdentity();
    return m;// GetTransformToParent();
  }

}

void DHNode::LoadData(ci::JsonTree &tree, Node *parent, const std::string &root_dir, size_t &idx){
  
  //set and increase the index
  idx_ = idx;
  idx++;

  //set the parent
  if (parent != 0x0){
    parent_ = dynamic_cast<DHNode *>(parent);
  }
  else{
    parent_ = 0x0;
  }

  //load the visual data (texture, vertices etc)
  try{
    LoadMeshAndTexture(tree, root_dir);
  }
  catch (ci::JsonTree::Exception &){

  }

  update_ = 0.0;

  //set up the dh param stuff
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
    n->LoadData(children[i], this, root_dir, idx);
    AddChild(n);
  }

}

void DHNode::createFixedTransform(const ci::Vec3f &axis, const float rads, ci::Matrix44f &output) const {

  output = ci::Matrix44f::createRotation(axis, rads);

}

void alternativeDerivativeTransform(const ci::Vec3f &axis, const float theta, ci::Matrix44f &deriv){

  deriv.setToIdentity();

  if (axis == ci::Vec3f(0, 1, 0)) {
    deriv.at(0, 0) = -sin(theta);
    deriv.at(0, 2) = cos(theta);
    deriv.at(2, 0) = -cos(theta);
    deriv.at(2, 2) = -sin(theta);
  }
  else if (axis == ci::Vec3f(0, -1, 0)){
    deriv.at(0, 0) = -sin(theta);
    deriv.at(0, 2) = -cos(theta);
    deriv.at(2, 0) = cos(theta);
    deriv.at(2, 2) = -sin(theta);
  }
  else{
    throw std::runtime_error("");
  }
}

ci::Matrix44f DHNode::GetDerivativeTransfromFromParent() const {

  ci::Matrix44f DH;
  DH.setToIdentity();

  if (type_ == Translation)
    glhDenavitHartenbergDerivative(a_, alpha_, d_ + update_, theta_, DH.m);
  else if (type_ == Rotation)
    glhDenavitHartenbergDerivative(a_, alpha_, d_, theta_ + update_, DH.m);
  else if (type_ == Fixed)
    glhDenavitHartenbergDerivative(a_, alpha_, d_, theta_, DH.m);
  else if (type_ == Alternative)
    alternativeDerivativeTransform(alt_axis_, theta_, DH);

  return DH;

}

ci::Matrix44f DHNode::GetTransformFromParent() const {
  
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
