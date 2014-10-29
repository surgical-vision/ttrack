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
  * Specifies a collection of points, a pose and intersection methods needed by the tracker.
  */
  class Model {

  public:

    /**
    * Construct the model from a configuration file.
    */
    explicit Model(const std::string &model_parameter_file);
    
    /**
    * Delete the model.
    */
    virtual ~Model(){}

    /**
    * Render the nodes that make up this model.
    */
    virtual void Render();
    
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
    void SetPose(const Pose &pose) { world_to_model_coordinates_ = pose; }

    /**
    * Get a single component for the model (only relevant if the model is made of several components). The index is 
    * currently quite arbitrary and is computed from the order in which the components are added.
    * @return The Node component wrapped in a Model.
    */
    virtual boost::shared_ptr<Model> GetComponent(const std::size_t component_idx) = 0;

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
    Model() {} 

    Node::Ptr model_; /**< A tree representation of the model as a sequence of coordinate systems with some attached geometry. */
    
    Pose world_to_model_coordinates_; /**< The transform from world coordinates (or camera) to the 'base' coordinates of the model. */
    
    cv::Vec3f principal_axis_; /**< The principal axis of the shape, this is not entirely meaningful for all shapes but can be useful for things shaped like cylinders. */

  };


 
}


#endif //_MODEL_HPP_
