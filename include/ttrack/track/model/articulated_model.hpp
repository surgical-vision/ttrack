#ifndef __ARTICULATED_MODEL_HPP__
#define __ARTICULATED_MODEL_HPP__

#include <cinder/Json.h>
#include <cinder/gl/gl.h>

#include "model.hpp"
#include "node.hpp"

namespace ttrk {

  /**
  * @class DenavitHartenbergArticulatedModel
  * @brief A specialisation of the Model to handle articulated models where the articulation is modelled with the DH parameters.
  * This class implements a transform function computing the SE3 to the coordinate system of each of it's articulated components using
  * DH transforms.
  */
  class DenavitHartenbergArticulatedModel : public Model {

  public:

    /**
    * Default constructor.
    * @param[in] model_parameter_file The file containing the locations of meshes and DH parameters for this model.
    * @param[in] save_file A file to save the model pose when requested.
    */
    DenavitHartenbergArticulatedModel(const std::string &model_parameter_file, const std::string &save_file);       
    
    /**
    * Constructor for testing etc when there is no save file.
    * @param[in] model_parameter_file The file containing the locations of meshes and DH parameters for this model.
    */
    explicit DenavitHartenbergArticulatedModel(const std::string &model_parameter_file);

    /**
    * Specialization of the JSON parsing function to enable the DH parameters to be loaded.
    * @param[in] jt The JSON tree containing the data.
    * @param[in] root_dir The directory containing the files of interest.
    */
    virtual void ParseJson(ci::JsonTree &jt, const std::string &root_dir);


  protected:

    /**
    * Default constructor.
    * @param[in] node The model node to build the component from.
    */
    //DenavitHartenbergArticulatedModel(Node::Ptr node, const ci::Matrix44f &world_to_model_transform) : Model(node, world_to_model_transform) {}


  };

}


#endif
