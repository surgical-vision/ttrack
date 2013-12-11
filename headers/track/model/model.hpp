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


  struct SurfacePoint {

    cv::Vec3f point_;
    cv::Mat descriptor_;
  
  };


  /**
  * @struct SimplePoint
  * @brief A really basic point-with-neighbours class to represent a
  * the elements of a meshed point cloud.
  */

  template<typename VertexType = cv::Vec3f>
  struct SimplePoint {
    
    /**
    * A default constructor for the struct.
    */ 
    SimplePoint(){}

    /**
    * Construct a point specifying its vertex.
    */
    SimplePoint(const VertexType &vertex){
      vertex_ = vertex;
    }

    /**
    * Add a neighbour to the point. 
    * @param[in] pt_index An index to the data structure where the points are stored.
    */    
    void AddNeighbour(size_t pt_index){ 
      neighbours_.push_back(pt_index);
    }

    //VertexType TransformPoint(const boost::math::quaternion<double> &rotation, const cv::Vec3d translation){
      //VertexType ret = *this;
    //}

    VertexType vertex_; /**< The vertex location. */
    std::vector<size_t> neighbours_; /**< The neighbours of the vertex. */

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
    Model(){}
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
    virtual std::vector<SimplePoint<> > Points(const Pose &pose) const = 0;
    virtual boost::shared_ptr<std::vector<Object *> > GetPoints(const Pose &pose);
    /**
    * Get an intersection between the model and a ray cast from the camera. This is needed in the tracking methods.
        * @param[in] ray The ray cast from the camera
    * @param[out] front The front intersection of the ray and the shape.
    * @param[out] back The back intersection of the ray and the shape. In the rare case that the intersection is exactly on an edge of the shape this will just be set to same as the front.
    * @return bool The success of the intersection test.
    */
    //virtual bool GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const = 0;
    bool ComputeAllIntersections(boost::shared_ptr<MonocularCamera> cam, const Pose &pose, const int rows, const int cols, cv::Mat &intersection_image) const;


    virtual cv::Vec3f PrincipalAxis() const = 0;

    virtual cv::Vec3f GetTrackedPoint() const = 0;

    boost::shared_ptr<BVH> bvh_;
    boost::shared_ptr<std::vector<Object *> > objects_;

  protected:


    std::vector<float> flattened_triangles_; 
    
    
    cv::Vec3f ReadVertexFromObj(std::ifstream &ifs) const;



    cv::Vec3i ReadFaceFromObj(std::ifstream &ifs) const;
    //virtual std::vector<Quadrilateral> ComputeBoundingBox(const Pose &pose) const = 0;
     
    /**
    *
    *
    */
    bool GetIntersectionTriangle(const cv::Vec3f &ray, const cv::Vec3f &vert0, const cv::Vec3f &vert1, const cv::Vec3f &vert2, float &distance) const ;
    bool GetIntersectionQuadrilateral(const cv::Vec3f &ray, const cv::Vec3f &vert0, const cv::Vec3f &vert1, const cv::Vec3f &vert2, const cv::Vec3f &vert3, float &distance) const;

    std::vector<SurfacePoint> surface_points_;
    std::vector<SimplePoint<> > model_points_;
  

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
    //MISTool(float radius, float height);
    MISTool(const std::string &model_file) ;
    
    /**
    * Provides access to a vector of SimplePoints which represent the cylinder.
    * @return The points.
    */
    virtual std::vector<SimplePoint<> > Points(const Pose &pose) const;
    
    /**
    * Finds intersection between a ray and the cylinder representing the tool.
    * @param[in] ray The ray projected from the camera center through some pixel.
    * @prarm[out] front The front intersection.
    * @param[out] back The back intersection.
    * @return bool The success of the intersection test
    */
    //virtual bool GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const;
    
    bool CircleIntersection(const cv::Vec3f &A, const cv::Vec3f &n, const cv::Vec3f &d, const float R,cv::Vec3f &intersection) const ;
    
    
    virtual cv::Vec3f PrincipalAxis() const { return cv::Vec3f(1,0,0); }

    float Length() { return height_tip_; }
    virtual float Radius() { return radius_; }
    float HeightFraction() { return height_fraction_; }
    virtual cv::Vec3f GetTrackedPoint() const;
  
  private:

    bool IntersectionInConvexHull(const std::vector<SimplePoint<> >&convex_hull, const cv::Vec3f &point) const;
    bool GetIntersectionPlane(const std::vector<SimplePoint<> >&points_in_plane, const cv::Vec3f &ray, cv::Vec3f &intersection) const;
    bool GetIntersectionPolygons(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const;
    //std::vector<std::vector<SimplePoint<> > > GetClasperPolygons(const Pose &pose) const;
    bool GetIntersectionShaft(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const;
    bool GetIntersectionTip(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const;

    float radius_; /**< The radius of the cylinder. */
    float height_; /**< The height of the cylinder. */
    //std::vector<SimplePoint<> > points_; /**< The representation of surface of the tool as a set of points which have references to their neighbours. */

    float radius_tip_;
    float height_tip_;

    float radius_fraction_;
    float height_fraction_;
    float height_curve_;
    float angle_curve_;
    float tip_offset_;

  };

};


#endif //_MODEL_HPP_
