#ifndef _MODEL_HPP_
#define _MODEL_HPP_
#include "../../headers.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "../pose.hpp"
#include "../../utils/primitives.hpp"
#include "../../utils/camera.hpp"
#include "fast_bvh/BVH.h"
#include "fast_bvh/Triangle.h"

namespace ttrk{

  /**
  * @struct CachedMesh
  * @brief As it is slow to recompute the mesh coordinates at a pose each time we need it, cache the most recent 
  * representation of it with the pose that was used to generate it and just return that if possible.
  */

  struct CachedMesh {

    boost::shared_ptr<std::vector<Object *> > mesh_;
    Pose pose_;

  };



  struct SurfacePoint {

    cv::Vec3d point_;
    cv::Mat descriptor_;

  };


  /**
  * @class Model
  * @brief An abstract class to represent an interface to trackable objects.
  * Specifies a collection of points, a pose and intersection methods needed by the tracker.
  */
  class Model {

  public:

    /**
    * Construct an empty model.
      */
    Model(const std::string &model_parameter_file);
    //Model &operator=(const Model &that){ return Model(); }
    Model(const Model &that){ }

    /**
    * Delete the model.
    */
    virtual ~Model(){}

    /**
    * Provides access to the points. As the model may be implemented as a parametrised shape or similar 
    * this is implemented as a virtual function to allow custom representation of the shape.
    * @return A vector of points.
    */
    virtual boost::shared_ptr<std::vector<Object *> > Points(const Pose &pose);
    
    /**
    * Get an intersection between the model and a ray cast from the camera. This is needed in the tracking methods. 
    * Warning: the mesh cache MUST be updated to the current pose using Model::UpdateMeshCache() before calling this function as it assumes that the mesh is stored with the correct pose. It is too slow to check for every call.
    * @param[in] ray The ray cast from the camera
    * @param[out] front The front intersection of the ray and the shape.
    * @param[out] back The back intersection of the ray and the shape. In the rare case that the intersection is exactly on an edge of the shape this will just be set to same as the front.
    * @return bool The success of the intersection test.
    */
    virtual bool GetIntersection(const cv::Vec3d &ray, cv::Vec3d &front, cv::Vec3d &back) const;

    virtual cv::Vec3d PrincipalAxis() const { return cv::Vec3d(1,0,0); }

    cv::Vec3d GetTrackedPoint() const { return tracked_point_; }
    cv::Vec3d SetTrackedPoint(const cv::Vec3d &tracked_point) { tracked_point_ = tracked_point ; }


  protected:

    void UpdateMeshCache(const Pose &pose);

    void ReadFromObjFile(const std::string &model_parameter_file);
    cv::Vec3d ReadVertexFromObj(std::ifstream &ifs) const;
    cv::Vec3i ReadFaceFromObj(std::ifstream &ifs) const;

    boost::shared_ptr<BVH> bvh_;
    CachedMesh cached_mesh_;

    std::vector<double> flattened_triangles_;     
    std::vector<SurfacePoint> surface_points_;

    cv::Vec3d tracked_point_;

  };


  /**
  * @class MISTool
  * @brief A simple implementation of a MIS tool that is built out of primitives
  * Subclass this to create a simple MIS Tool 
  */
  class MISTool : public Model {

  public:
    
    /**
    * Construct a cylinder using its minimal parameters.
    * @param[in] radius The cylinder radius.
    * @param[in] height The cylinder height.
    */
    MISTool(const std::string &model_file) : Model(model_file) {}
          
  };

};


#endif //_MODEL_HPP_
