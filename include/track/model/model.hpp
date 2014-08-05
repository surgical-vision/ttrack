#ifndef _MODEL_HPP_
#define _MODEL_HPP_



#include <cinder/TriMesh.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Texture.h>
#include <utility>
#include <boost/tuple/tuple.hpp>

#include "../../headers.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#include "../pose.hpp"
#include "../../utils/primitives.hpp"
#include "../../utils/camera.hpp"


namespace ttrk{

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
    
    /**
    * Delete the model.
    */
    virtual ~Model(){}

    
    typedef boost::tuple<boost::shared_ptr<ci::TriMesh>, boost::shared_ptr<ci::gl::Texture>, ci::Matrix44d> MeshTextureAndTransform;
    //typedef std::pair< boost::shared_ptr<ci::TriMesh> , ci::Matrix44d > MeshAndTransform;
    
    /**
    * Get an intersection between the model and a ray cast from the camera. This is needed in the tracking methods. 
    * Warning: the mesh cache MUST be updated to the current pose using Model::UpdateMeshCache() before calling this function as it assumes that the mesh is stored with the correct pose. It is too slow to check for every call.
    * @param[in] ray The ray cast from the camera
    * @param[out] front The front intersection of the ray and the shape.
    * @param[out] back The back intersection of the ray and the shape. In the rare case that the intersection is exactly on an edge of the shape this will just be set to same as the front.
    * @return bool The success of the intersection test.
    */
    virtual bool GetIntersection(const cv::Vec3d &ray, cv::Vec3d &front, cv::Vec3d &back) const { throw(std::runtime_error("Not implemented!\n")); return true; }

    virtual cv::Vec3d PrincipalAxis() const { return cv::Vec3d(1,0,0); }

    cv::Vec3d GetTrackedPoint() const { return tracked_point_; }
    cv::Vec3d SetTrackedPoint(const cv::Vec3d &tracked_point) { tracked_point_ = tracked_point ; }

    virtual std::vector< MeshTextureAndTransform > GetRenderableMeshes() { ci::Matrix44d eye; eye.setToIdentity(); std::vector<MeshTextureAndTransform> v; v.push_back(MeshTextureAndTransform(model_,texture_,eye)); return v; }

  protected:

    Model() {} // load nothing - only useful for the derived classes which need to delay loading

    cv::Vec3d tracked_point_;
    boost::shared_ptr<ci::TriMesh> model_;
    boost::shared_ptr<ci::gl::Texture> texture_;
    
  };
 
}


#endif //_MODEL_HPP_
