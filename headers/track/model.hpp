#ifndef _MODEL_HPP_
#define _MODEL_HPP_
#include "../headers.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "pose.hpp"

namespace ttrk{

  /**
  * @struct SimplePoint
  * @brief A really basic point-with-neighbours class to represent a
  * the elements of a connected point cloud.
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

    VertexType TransformPoint(const boost::math::quaternion<double> &rotation, const cv::Vec3d translation){
      VertexType ret = *this;
    }

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
    
    /**
    * Delete the model.
    */
    virtual ~Model(){}
    
    /**
    * Provides access to the points. As the model may be implemented as a parametrised shape or similar 
    * this is implemented as a virtual function to allow custom representation of the shape.
    * @return A vector of points.
    */
    virtual std::vector<SimplePoint<> > Points() const = 0;
    
    /**
    * Get an intersection between the model and a ray cast from the camera. This is needed in the tracking methods.
    * @param[in] ray The ray cast from the camera
    * @param[out] front The front intersection of the ray and the shape.
    * @param[out] back The back intersection of the ray and the shape. In the rare case that the intersection is exactly on an edge of the shape this will just be set to same as the front.
    */
    virtual void GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const = 0;
    
    

  protected:
  
 

  };


  /**
  * @class MISTool
  * @brief An implementation of a MIS tool.
  * Provide a simple representation of an MIS tool as a cylinder.
  */
  class MISTool : public Model {

  public:
    
    /**
    * Construct a cylinder using its minimal parameters.
    * @param[in] radius The cylinder radius.
    * @param[in] height The cylinder height.
    */
    MISTool(int radius, int height);
    
    /**
    * Provides access to a vector of SimplePoints which represent the cylinder.
    * @return The points.
    */
    virtual std::vector<SimplePoint<> > Points() const;
    //virtual std::vector<SimplePoint<> > TransformedPoints() const ;
    /**
    * Finds intersection between a ray and the cylinder representing the tool.
    * @param[in] ray The ray projected from the camera center through some pixel.
    * @prarm[out] front The front intersection.
    * @param[out] back The back intersection.
    */
    virtual void GetIntersection(const cv::Vec3f &ray, cv::Vec3f &front, cv::Vec3f &back, const Pose &pose) const;

  private:

    int radius_; /**< The radius of the cylinder. */
    int height_; /**< The height of the cylinder. */
    //std::vector<SimplePoint<> > points_; /**< The representation of surface of the tool as a set of points which have references to their neighbours. */

  };

};


#endif //_MODEL_HPP_