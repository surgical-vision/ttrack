#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <cinder/Json.h>
#include <cinder/TriMesh.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/Vbo.h>
#include <utility>
#include <boost/tuple/tuple.hpp>

#include "../../headers.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#include "../pose.hpp"
#include "../../utils/camera.hpp"
#include "node.hpp"

namespace ttrk{

  /**
  * @class Model
  * @brief An abstract class to represent an interface to trackable objects.
  * Specifies a collection of vertices/mesh, a pose and intersection methods needed by the tracker.
  */
  class Model {

  public:

    /**
    * Construct the model from a configuration file.
    * @param[in] model_parameter_file The configuration file for the model.
    * @param[in] save_file A file to save the model pose when requested.
    */
    explicit Model(const std::string &model_parameter_file, const std::string &save_file);
    
    /**
    * Delete the model.
    */
    virtual ~Model(){}

    /**
    * Render the nodes that make up this model with a bound texture.
    * @id The id of the texture. Use zero for the default target.
    */
    virtual void RenderTexture(int id);

    /**
    * Render the nodes that make up this model with a material rather than a texture.
    */
    virtual void RenderMaterial();
    
    /**
    * Get the principal axis of the Model. This may not be entirely meaningful for all shapes.
    * @return The principal axis in model coordinates.
    */
    cv::Vec3f PrincipalAxis() const { return principal_axis_; }

    /**
    * Set the principal axis of the Model. This may not be entirely meaningful for all shapes.
    * @param[in] p_axis The principal axis in model coordinates.
    */
    void SetPrincipalAxis(const cv::Vec3f &p_axis) { principal_axis_ = p_axis; }

    /**
    * Set the principal axis of the Model. This may not be entirely meaningful for all shapes.
    * @param[in] pose The new model pose.
    */
    void SetBasePose(const Pose &pose) { world_to_model_coordinates_ = pose; }

    /**
    * Get the current world to model coordinate system pose.
    * @param[in] pose The new model pose.
    */
    Pose GetBasePose() const { return world_to_model_coordinates_; }

    /**
    * Get the pose for a single component for the model (returns the base pose if the model is only one component). The index is 
    * currently quite arbitrary and is computed from the order in which the components are added.
    * @param[in] component_idx The index of the component. The indexes are quite an arbitrary choice as there's no obvious way to number tree nodes (AFAIK).
    * @return The Pose of the request node in the world coordinate system.
    */
    Pose GetComponentPose(const std::size_t component_idx) { return Pose(model_->GetChildByIdx(component_idx)->GetWorldTransform(world_to_model_coordinates_)); }

    /**
    * Update the pose of the model. For a rigid model this just updates the 6/7 dofs of the camera to model coordinate transform. If there are articulated components then this vector updates them too. The order of the updates should be the same order that they come out of the ComputeJacobian function.
    * @param[in] updates The scalar updates to each degree of freedom.
    */
    virtual void UpdatePose(std::vector<float> &updates); 

    /**
    * Set the pose parameters of the model.
    * @param[in] pose The value of each degree of freedom.
    */
    virtual void SetPose(std::vector<float> &pose);

    /**
    * Get the pose parameters of the model.
    * @param[out] pose The value of each degree of freedom.
    */
    virtual void GetPose(std::vector<float> &pose);

    /**
    * Compute the Jacobian of the pose w.r.t some point.
    * @param[in] point The 3D point used to compute the jacobian.
    * @return The jacobian in the form \f$ \mathbf{J} = \bigg[ \big(\frac{\partial X}{\partial \lambda_{0}},\frac{\partial Y}{\partial \lambda_{0}},\frac{\partial Z}{\partial \lambda_{0}}\big), \big(\frac{\partial X}{\partial \lambda_{1}},\frac{\partial Y}{\partial \lambda_{1}},\frac{\partial Z}{\partial \lambda_{1}}\big), ... \big(\frac{\partial X}{\partial \lambda_{n}},\frac{\partial Y}{\partial \lambda_{n}},\frac{\partial Z}{\partial \lambda_{n}}\big) \bigg] \f$
    */
    virtual std::vector<ci::Vec3f> ComputeJacobian(const ci::Vec3f &point, const int target_frame_idx) const;

    /**
    * Compute the Jacobian of the pose w.r.t some point.
    * @param[in] point The 3D point used to compute the jacobian.
    * @return The jacobian in the form \f$ \mathbf{J} = \bigg[ \big(\frac{\partial X}{\partial \lambda_{0}},\frac{\partial Y}{\partial \lambda_{0}},\frac{\partial Z}{\partial \lambda_{0}}\big), \big(\frac{\partial X}{\partial \lambda_{1}},\frac{\partial Y}{\partial \lambda_{1}},\frac{\partial Z}{\partial \lambda_{1}}\big), ... \big(\frac{\partial X}{\partial \lambda_{n}},\frac{\partial Y}{\partial \lambda_{n}},\frac{\partial Z}{\partial \lambda_{n}}\big) \bigg] \f$
    */
    std::vector<ci::Vec3f> ComputeJacobian(const cv::Vec3f &point, const int target_frame_idx) const { return ComputeJacobian(ci::Vec3f(point[0], point[1], point[2]), target_frame_idx); }

    Node::Ptr GetModel() { return model_; } //will remove this?

    const Node::Ptr GetModel() const { return model_; }

    /**
    * Serialize the pose and write it to the file (see constructor).
    */
    void WritePoseToFile();

    /**
    * Get the total number of models as a string, useful for saving etc.
    * @return The total number of models as a string.
    */
    static std::string GetCurrentModelCount() { std::stringstream ss; ss << total_model_count_; return ss.str(); }

    /**
    * Get a reference to the model's texture (for binding etc).
    * @return The texture.
    */
    ci::gl::Texture &GetTexture() { return model_->GetTexture(); }

  protected:

    /**
    * Construct a model from a single component of the model - useful for situations where the model is made up of several components.
    * @param[in] component The Model component to use in constructing this instance.
    * @param[in] world_to_model_transform
    */
    Model(Node::Ptr component, const ci::Matrix44f &world_to_model_transform);

    /**
    * Load the model from a config file.
    * @param[in] filename The filepath to the config file.
    */
    virtual void LoadFromFile(const std::string &filename);

    /**
    * Load the model from a JSON config file.
    * @param[in] filename The filepath to the config file.
    */
    void LoadJson(const std::string &filename);

    /**
    * Parse the JSON file and load the data for each node.
    * @param[in] tree The JSON tree containing the data.
    * @param[in] root_dir The directory containing the data to load.
    */
    virtual void ParseJson(ci::JsonTree &tree, const std::string &root_dir) = 0;


    /**
    * Load nothing - only useful for the derived classes which need to delay loading
    */
    Model() { total_model_count_++; }

    Node::Ptr model_; /**< A tree representation of the model as a sequence of coordinate systems with some attached geometry. */
    
    Pose world_to_model_coordinates_; /**< The transform from world coordinates (or camera) to the 'base' coordinates of the model. */
    
    cv::Vec3f principal_axis_; /**< The principal axis of the shape, this is not entirely meaningful for all shapes but can be useful for things shaped like cylinders. */

    std::ofstream ofs_; /**< A file to save the pose of the model at each frame. */

    static size_t total_model_count_; /** Count of all created models so when we create a new one it gets it's own file. */

  };


 
}


#endif //_MODEL_HPP_
