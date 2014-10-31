#ifndef __NODE_HPP__
#define __NODE_HPP__

#include <cinder/Json.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Vbo.h>

namespace ttrk {

  /**
  * @class Node
  * @brief An single node in the tree representation of an articulated model
  * Each element in the tree model maintains a list of 'child' nodes it's connected to and can compute transformations to its
  * coordinate systems.
  */
  class Node {

  public:
    
    /**
    * @typedef Get a pointer to this type.
    */
    typedef boost::shared_ptr<Node> Ptr;
    
    /**
    * Pure virtual function for loading the data. Overwritten in the implementations of Node to handle different loading types.
    * @param[in] tree The JSON tree that we will parse to get the files and the children.
    * @param[in] parent The parent of this node.
    * @param[in] root_dir The root directory where the files are stored.
    */
    virtual void LoadData(ci::JsonTree &tree, Node::Ptr parent, const std::string &root_dir) = 0;
    
    /**
    * Get the transform between the world coordinate system and this node.
    * @param[in] base_frame_pose The transform from World Coordinates to the base frame of the object.
    * @return The world transform in a 4x4 float matrix.
    */
    virtual ci::Matrix44f GetWorldTransform(const ci::Matrix44f &base_frame_pose) const = 0;
    
    /**
    * Get the transform between the this node and the parent node.
    * @return The transform in a 4x4 float matrix.
    */
    virtual ci::Matrix44f GetRelativeTransform() const = 0;
    
    /**
    * Add a child node to this node.
    * @param[in] child The child node to add.
    */
    void AddChild(Node::Ptr child) { children_.push_back(child); }

    /**
    * Render a node element, will recursively call Render on all child nodes.
    */
    void Render();

    /**
    * Compute the jacobian for a 3D point (passed in world coordinates) with respect to this coordinate frame.
    * This will recursively call the parent frames right up to the base frame and return a 3-vector for each.
    * @param[in] point The 3D point in the target coordinate frame (usually camera).
    * @param[in] jacobian The jacobian which is added to by each of the joints. This needs to be a reference pass as it's filled in base to tip (roughly).
    */
    virtual void ComputeJacobianForPoint(const ci::Vec3f &point, std::vector<ci::Vec3f> &jacobian);

    /**
    * Update the pose of the model using the jacobians (with whatever cost function modification).
    * @param[in] updates The update vector iterator, there should be N for the rigid base part of the model (probably 6-7) and then one for each component of the articulated components (if there are any). The order is supposed to be the same as the order they came out from ComputeJacobians.
    */
    virtual void UpdatePose(std::vector<float>::iterator &updates) = 0;

    Node::Ptr GetParent() { return parent_; }
    std::vector< Node::Ptr> GetChildren() { return children_; }
    Node::Ptr GetChildByIdx(std::size_t &curr_idx, const std::size_t target_idx);

  protected:

    /**
    * Load the mesh and texture from the JSON file.
    * @param[in] tree The JSON tree that we will parse to get the files and the children.
    * @param[in] root_dir The root directory where the files are stored.
    */
    void LoadMeshAndTexture(ci::JsonTree &tree, const std::string &root_dir);

    Node::Ptr parent_; /**< This node's parent. */
    std::vector< Node::Ptr > children_; /**< This node's children. */

    ci::TriMesh model_; /**< The 3D mesh that the model represents. */
    ci::gl::VboMesh	vbo_; /**< VBO to store the model for faster drawing. */
    ci::gl::Texture texture_; /**< The texture for the model. */

  };

  /**
  * @class DHNode
  * @brief A specialization of the node type to incorporate nodes which are connected by DH specified transforms.
  * A specialization of the node type to incorporate nodes which are connected by DH specified transforms rather than SE3/SO3.
  */
  class DHNode : public Node {

  public:

    /**
    * @typedef Get a pointer to this type.
    */
    typedef boost::shared_ptr<DHNode> Ptr;

    /**
    * Specialization of the load function to handle JSON files which specify DH parameters.
    * @param[in] tree The JSON tree that we will parse to get the files and the children.
    * @param[in] parent The parent of this node.
    * @param[in] root_dir The root directory where the files are stored.
    */
    virtual void LoadData(ci::JsonTree &tree, Node::Ptr parent, const std::string &root_dir);

    /**
    * Get the transform between the world coordinate system and this node using the DH transform.
    * @param[in] base_frame_pose The transform from World Coordinates to the base frame of the object.
    * @return The world transform in a 4x4 float matrix.
    */
    virtual ci::Matrix44f GetWorldTransform(const ci::Matrix44f &base_frame_pose) const;

    /**
    * Get the transform between the this node and the parent node using the DH transforms.
    * @return The transform in a 4x4 float matrix.
    */
    virtual ci::Matrix44f GetRelativeTransform() const;

    /**
    * Update the pose of the model using the jacobians (with whatever cost function modification).
    * @param[in] updates The update vector, there should be N for the rigid base part of the model (probably 6-7) and then one for each component of the articulated components (if there are any). The order is supposed to be the same as the order they came out from ComputeJacobians.
    */
    virtual void UpdatePose(std::vector<float>::iterator &updates);

  protected:

    ci::Matrix44f ComputeDHTransform() const;

    enum JointType { Rotation, Translation, Fixed };

    JointType type_; /**< The joint type of the DH element. Dictates whether the update is applied to d or @theta. */
    float alpha_; /**< @alpha in the DH parameter set. Angle about the common normal between links. */
    float theta_; /**< @theta in the DH parameter set. Angle about the previous joint axis. */
    float a_; /**< a in the DH parameter set. Length of the common normal. */
    float d_; /**< d in the DH parameter set. Offset along previous joint axis to the common normal. */
    float update_; /**< The update to the DH set to compute the transformation. */

  };

}


#endif