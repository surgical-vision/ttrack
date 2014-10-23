#include <cinder/ObjLoader.h>
#include <cinder/gl/Texture.h>
#include <cinder/app/App.h>
#include <cinder/gl/Light.h>

#include "../../../include/track/model/node.hpp"

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

ci::Matrix44f DHNode::GetWorldTransform() const {
  throw std::runtime_error("Not implemented");
  return ci::Matrix44f();
}

ci::Matrix44f glhMultMatrixRight(const ci::Matrix44f &A, ci::Matrix44f &B);

ci::Matrix44f DHNode::GetRelativeTransform() const {

  if (parent_ != 0x0){
    DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
    //return p->ComputeDHTransform() * ComputeDHTransform();
    return glhMultMatrixRight(p->GetRelativeTransform(), ComputeDHTransform());
  }
  else{
    return ComputeDHTransform();
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

// OpenGL matrix offsets
#define _00 0
#define _10 1
#define _20 2
#define _30 3
#define _01 4
#define _11 5
#define _21 6
#define _31 7
#define _02 8
#define _12 9
#define _22 10
#define _32 11
#define _03 12
#define _13 13
#define _23 14
#define _33 15

ci::Matrix44f glhMultMatrixRight(const ci::Matrix44f &A, ci::Matrix44f &B)
{
  
  GLfloat M[16];
  M[_00] = B[_00] * A[_00] + B[_01] * A[_10] + B[_02] * A[_20] + B[_03] * A[_30];
  M[_10] = B[_10] * A[_00] + B[_11] * A[_10] + B[_12] * A[_20] + B[_13] * A[_30];
  M[_20] = B[_20] * A[_00] + B[_21] * A[_10] + B[_22] * A[_20] + B[_23] * A[_30];
  M[_30] = B[_30] * A[_00] + B[_31] * A[_10] + B[_32] * A[_20] + B[_33] * A[_30];
  M[_01] = B[_00] * A[_01] + B[_01] * A[_11] + B[_02] * A[_21] + B[_03] * A[_31];
  M[_11] = B[_10] * A[_01] + B[_11] * A[_11] + B[_12] * A[_21] + B[_13] * A[_31];
  M[_21] = B[_20] * A[_01] + B[_21] * A[_11] + B[_22] * A[_21] + B[_23] * A[_31];
  M[_31] = B[_30] * A[_01] + B[_31] * A[_11] + B[_32] * A[_21] + B[_33] * A[_31];
  M[_02] = B[_00] * A[_02] + B[_01] * A[_12] + B[_02] * A[_22] + B[_03] * A[_32];
  M[_12] = B[_10] * A[_02] + B[_11] * A[_12] + B[_12] * A[_22] + B[_13] * A[_32];
  M[_22] = B[_20] * A[_02] + B[_21] * A[_12] + B[_22] * A[_22] + B[_23] * A[_32];
  M[_32] = B[_30] * A[_02] + B[_31] * A[_12] + B[_32] * A[_22] + B[_33] * A[_32];
  M[_03] = B[_00] * A[_03] + B[_01] * A[_13] + B[_02] * A[_23] + B[_03] * A[_33];
  M[_13] = B[_10] * A[_03] + B[_11] * A[_13] + B[_12] * A[_23] + B[_13] * A[_33];
  M[_23] = B[_20] * A[_03] + B[_21] * A[_13] + B[_22] * A[_23] + B[_23] * A[_33];
  M[_33] = B[_30] * A[_03] + B[_31] * A[_13] + B[_32] * A[_23] + B[_33] * A[_33];
  for (unsigned int i = 0; i < 16; ++i)
    B[i] = M[i];

  return B;
}

void glhDenavitHartenberg(GLfloat a, GLfloat alpha, GLfloat d, GLfloat theta, GLfloat* A) {

  GLfloat sa = sin(alpha);
  GLfloat ca = cos(alpha);
  GLfloat st = sin(theta);
  GLfloat ct = cos(theta);

  A[_00] = ct;
  A[_10] = ca * st;
  A[_20] = sa * st;
  A[_30] = 0.0;
  A[_01] = -st;
  A[_11] = ca * ct;
  A[_21] = sa * ct;
  A[_31] = 0.0;
  A[_02] = 0.0;
  A[_12] = -sa;
  A[_22] = ca;
  A[_32] = 0.0;
  A[_03] = a;
  A[_13] = -sa * d;
  A[_23] = ca * d;
  A[_33] = 1.0;

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