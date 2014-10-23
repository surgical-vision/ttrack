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
    * Get an intersection between the model and a ray cast from the camera. This is needed in the tracking methods. 
    * Warning: the mesh cache MUST be updated to the current pose using Model::UpdateMeshCache() before calling this function as it assumes that the mesh is stored with the correct pose. It is too slow to check for every call.
    * @param[in] ray The ray cast from the camera
    * @param[out] front The front intersection of the ray and the shape.
    * @param[out] back The back intersection of the ray and the shape. In the rare case that the intersection is exactly on an edge of the shape this will just be set to same as the front.
    * @return bool The success of the intersection test.
    */
    //virtual bool GetIntersection(const cv::Vec3d &ray, cv::Vec3d &front, cv::Vec3d &back) const = 0;

    virtual cv::Vec3d PrincipalAxis() const { return cv::Vec3d(1,0,0); }

  protected:

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
    void InitGL();

    Model() {} // load nothing - only useful for the derived classes which need to delay loading

    Node::Ptr model_; /**< A tree representation of the model as a sequence of coordinate systems with some attached geometry. */
    Pose world_to_model_coordinates_; /**< The transform from world coordinates (or camera) to the 'base' coordinates of the model. */
    
  };
 
}


#endif //_MODEL_HPP_
