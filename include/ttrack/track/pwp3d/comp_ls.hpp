#ifndef __COMP_LS_HPP__
#define __COMP_LS_HPP__

#include "stereo_pwp3d.hpp"

namespace ttrk {

  const int NUM_COMPONENTS = 2;

  class ComponentLevelSet : public StereoPWP3D {

  public:

    ComponentLevelSet(boost::shared_ptr<StereoCamera> camera);
    ~ComponentLevelSet();

    virtual void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error);

    /**
    * Load the shaders we use to compute the projections and contours for the pose estimation.
    */
    void LoadShaders();

    /**
    * Construct a signed distance function and the intersection images which hold the 3D points in camera space which each point on the image plane projects to. Points outside the contour project to GL_FAR.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model used for the projection.
    * @param[out] sdf_image A 32 bit single channel floating point image of the distance from each pixel to the contour. Positive values inside contour and negative values outside.
    * @param[out] front_intersection_image A 32 bit 3 channel image of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] front_intersection_image A 32 bit 3 channel image of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    */
    void ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);

    /**
    * Render the mesh in the current pose getting the depth of the each pixel and the outer contour.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model to use for the projection.
    * @param[out] front_depth A 32 bit single channel image of the depth of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] back_depth A 32 bit single channel image of the depth of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] contour An 8 bit single channel image which is 0 at every point that is not on the outer contour of the projected mesh and 255 where it is.
    */
    void RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth, cv::Mat &contour);


  protected:

    ci::gl::Fbo component_map_framebuffer_;

    ci::gl::GlslProg component_shader_;

  };


}

#endif