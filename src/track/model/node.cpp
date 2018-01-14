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

  if (tree.hasChild("obj-file-hack")){
    obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file-hack"].getValue<std::string>());
    loader = ci::ObjLoader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()), true);
    loader.load(&model_hack_, true, true, false);
    vbo_hack_ = ci::gl::VboMesh(model_hack_);
  }


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


void Node::RenderTextureHack_FORCLASPERS(int id, const ci::Matrix44f world_transform){

  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelView(this->GetWorldTransform(world_transform));

    if (texture_){
      if (id == 0)
        texture_.enableAndBind();
      else
        texture_.bind(id);

    }

    if (model_hack_.getNumVertices() != 0)
      ci::gl::draw(vbo_hack_);
    else if (model_.getNumVertices() != 0)
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
    children_[i]->RenderTextureHack_FORCLASPERS(id, world_transform);
  }

}


void Node::RenderLines(const ci::Matrix44f &world_transform) const {

  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelView(this->GetWorldTransform(world_transform));

    ci::Vec3f end(0, 0, 0);
    if (this->idx_ == 0)
      end = ci::Vec3f(0, 0, -40);
    else if (idx_ == 1)
      end = ci::Vec3f(11, 0, 0);
    else if (idx_ == 4){
      end = ci::Vec3f(0, 0, 9.2);
    }
    else if (idx_ == 5){
      end = ci::Vec3f(0, 0, 9.2);
    }

    ci::gl::drawLine(ci::Vec3f(0, 0, 0), end);

    ci::gl::popModelView();
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->RenderLines(world_transform);
  }

}

void Node::RenderTextureHack(int id, const ci::Matrix44f world_transform){

  if (drawing_flag_){

    ci::gl::pushModelView();
    
    ci::gl::multModelView(this->GetWorldTransform(world_transform));

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
    children_[i]->RenderTextureHack(id, world_transform);
  }

}

void Node::RenderTexture(int id){
  
  if (drawing_flag_){

    ci::gl::pushModelView();

    /*ci::Matrix44f correct_transform = GetRelativeTransformToRoot();


    Node *par = parent_;
    if (par != 0x0){

      for (;;){
        if (par->parent_ == 0x0) break;
        par = par->parent_;
      }

      if (par->idx_ != 0) throw std::runtime_error("");

      ci::Matrix44f pp1 = par->GetRelativeTransformToRoot();
      ci::Matrix44f pp2 = GetRelativeTransformFromNodeToNodeByIdx(0, idx_);

      if (pp1 != ci::Matrix44f()){
        ci::app::console() << "pp1 != eye" << std::endl;
        ci::app::console() << "pp1 = " << pp1 << "\n" << "eye = " << ci::Matrix44f() << std::endl;
      }
      else{
        ci::app::console() << "pp1 == eye" << std::endl;
      }

      if (pp2 != correct_transform){
        ci::app::console() << "pp2 != correct_transform" << std::endl;
        ci::app::console() << "pp2 = " << pp2 << "\n" << "correct_transform = " << correct_transform << std::endl;
      }
      else{
        ci::app::console() << "pp2 != correct_transform" << std::endl;
      }


      ci::Matrix44f m1 = GetRelativeTransformFromNodeToNodeByIdx(0,1);
      Node *n1 = par->GetChildByIdx(1);
      ci::Matrix44f m11 = n1->GetRelativeTransformToRoot();
      if (m11 != m1){
        ci::app::console() << "m1 != m11" << std::endl;
        ci::app::console() << "m1 = " << m1 << "\n" << "m11 = " << m11 << std::endl;
      }
      else{
        ci::app::console() << "m1 == m11" << std::endl;
      }

      ci::Matrix44f m2 = GetRelativeTransformFromNodeToNodeByIdx(0, 2);
      Node *n2 = par->GetChildByIdx(2);
      ci::Matrix44f m22 = n2->GetRelativeTransformToRoot();
      if (m22 != m2){
        ci::app::console() << "m2 != m22" << std::endl;
        ci::app::console() << "m2 = " << m2 << "\n" << "m22 = " << m22 << std::endl;
      }
      else{
        ci::app::console() << "m2 == m22" << std::endl;
      }

      ci::Matrix44f m3 = GetRelativeTransformFromNodeToNodeByIdx(0, 3);
      Node *n3 = par->GetChildByIdx(3);
      ci::Matrix44f m33 = n3->GetRelativeTransformToRoot();
      if (m33 != m3){
        ci::app::console() << "m3 != m33" << std::endl;
        ci::app::console() << "m3 = " << m3 << "\n" << "m33 = " << m33 << std::endl;
      }
      else{
        ci::app::console() << "m3 == m33" << std::endl;
      }

      ci::Matrix44f m4 = par->GetRelativeTransformFromNodeToNodeByIdx(0, 4);
      Node *n4 = par->GetChildByIdx(4);
      ci::Matrix44f m44 = n4->GetRelativeTransformToRoot();
      if (m44 != m4){
        ci::app::console() << "m4 != m44" << std::endl;
        ci::app::console() << "m4 = " << m4 << "\n" << "m44 = " << m44 << std::endl;
      }
      else{
        ci::app::console() << "m4 == m44" << std::endl;
      }

    }*/

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


std::vector< Node::Ptr > Node::GetAllChildren() {

  std::vector<Node::Ptr> all_children = children_;
  for (auto &child : children_){
    auto &cc = child->GetAllChildren();
    all_children.insert(all_children.end(), cc.begin(), cc.end());
  }
  return all_children;

}

std::vector< Node::ConstPtr > Node::GetAllChildren() const{
  
  std::vector<Node::ConstPtr> all_children;
  for (auto &child : children_){
    all_children.push_back(child);
    auto &cc = child->GetAllChildren();
    all_children.insert(all_children.end(), cc.begin(), cc.end());
  }
  return all_children;

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

void Node::ComputeJacobianForPoint(const ci::Matrix44f &world_transform, const ci::Vec3f &point_in_camera_coords, const int target_frame_index, std::vector<ci::Vec3f> &jacobian) const {

  int idx = idx_;

  //the node with null parent is the root node so it's pose is basically just the base pose.
  if (parent_ != nullptr && idx_ != 3){ //3 is the fixed node so there's no derivative for it.

    if (!NodeIsChild(target_frame_index)){// || !NodeIsTransformable() ){

      jacobian.push_back(ci::Vec3f(0.0f, 0.0f, 0.0f));

    }
    //else if (idx_ == 2){
    //  jacobian.push_back(ci::Vec3f(0.0f, 0.0f, 0.0f));
    //}
    else{

      //ci::Vec3f point_in_target_frame_coords = (GetChildByIdx(target_frame_index)->GetWorldTransform(world_transform)).inverted() * point_in_camera_coords;
      ci::Vec3f point_in_target_frame_coords = (GetWorldTransform(world_transform)).inverted() * point_in_camera_coords;
      //get the transform from this node to the node where the target point lives
      //ci::Matrix44f T_1 = world_transform * parent_->GetRelativeTransformToRoot();

      ci::Matrix44f T_3;
      if (idx_ == target_frame_index){
        T_3.setToIdentity();
      }
      else{
       T_3 = GetRelativeTransformFromNodeToNodeByIdx(idx_, target_frame_index);
      }

      //point_in_target_frame_coords = T_3.inverted() * point_in_target_frame_coords;


      //parent to node transform is in T_3. does not need to be applied again.
      ci::Vec3f z;
      if (idx_ == 4){
        z = ci::Vec3f(0, 1, 0);// , 1);
      }
      else if (idx_ == 5){
        z = ci::Vec3f(0, -1, 0);// , 1); //this one is broken!
      }
      else{
        z = ci::Vec3f(GetAxis());// , 1);
      }
      
      //ci::Vec4f end = T_3 * ci::Vec4f(point_in_target_frame_coords, 1);
      
      ci::Vec3f end = z.cross(point_in_target_frame_coords);
      
      //ci::Matrix44f T_frame_to_world = GetChildByIdx(target_frame_index)->GetWorldTransform(world_transform);
      ci::Matrix44f T_frame_to_world = GetChildByIdx(idx_)->GetWorldTransform(world_transform);

      //ci::Vec4f jac = T_1 * (z.cross(end));
      ci::Vec3f jac = T_frame_to_world.subMatrix33(0,0) * end;

      jacobian.push_back(ci::Vec3f(jac[0], jac[1], jac[2]));

    }

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->ComputeJacobianForPoint(world_transform, point_in_camera_coords, target_frame_index, jacobian);
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

  Node::ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  return;

  //if (target_frame_index == 0){

  //  jacobian[7 + idx_] = ci::Vec3f(0.0f,0.0f,0.0f);

  //}

  //else if (target_frame_index == 1){

  //  ComputeJacobianForHead(world_transform, point, jacobian);

  //}

  //else if (target_frame_index == 4 || target_frame_index == 5){

  //  ComputeJacobianForClasperYaw(world_transform, point, jacobian);
  //  ComputeJacobianForClasperRotate(world_transform, point, target_frame_index, jacobian);

  //}
  //
  //else{

  //  jacobian[7 + idx_] = ci::Vec3f(0.0f, 0.0f, 0.0f);

  //}

  //for (size_t i = 0; i < children_.size(); ++i){
  //  children_[i]->ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  //}

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

ci::Matrix44f DHNode::GetRelativeTransformFromNodeToNodeByIdx(const int start_idx, const int end_idx) const {

  if (end_idx <= start_idx) {
    ci::app::console() << "Error, start_idx should be less than end idx" << std::endl;
    throw(std::runtime_error(""));
  }

  const Node *root_node = 0x0;
  if (parent_ == 0x0){
    root_node = this;
  }
  else{
    root_node = parent_;
    for (;;){
      if (root_node->GetParent() == 0x0){
        break;
      }
      root_node = root_node->GetParent();
    }

    if (root_node->GetIdx() != 0){
      ci::app::console() << "Error, root node index is not zero!" << std::endl;
      throw std::runtime_error("");
    }
  }

  const Node *start_node = root_node->GetChildByIdx(start_idx);
  const Node *end_node = root_node->GetChildByIdx(end_idx);

  const ci::Matrix44f root_to_start = start_node->GetRelativeTransformToRoot();
  const ci::Matrix44f root_to_end = end_node->GetRelativeTransformToRoot();

  return root_to_start.inverted() * root_to_end;

  //  //before we get to the target node, return the transform to the parent * recursive call up the chain to parent
  //else{
  //  if (parent_ != 0x0){
  //    DHNode *p = dynamic_cast<DHNode *>(parent_);
  //    return glhMultMatrixRight(GetTransformFromParent(), p->GetRelativeTransformToNodeByIdx(target_idx));
  //  }
  //  else{
  //    ci::Matrix44f m;
  //    m.setToIdentity();
  //    return m;
  //  }
  //}


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

  ci::app::console() << "SHould not call this function: alternativeDerivativeTransform()" << std::endl;
  throw std::runtime_error("");

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

  ci::app::console() << "SHould not call this function: getDerivativeTransformFromParent()" << std::endl;
  throw std::runtime_error("");

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
