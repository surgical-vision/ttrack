#ifndef __COMP_LS_HPP__
#define __COMP_LS_HPP__

#include "stereo_pwp3d.hpp"

namespace ttrk {

  const int NUM_COMPONENTS = 2;

  class ComponentLevelSet : public StereoPWP3D {

  public:

    ComponentLevelSet(boost::shared_ptr<StereoCamera> camera);
    ~ComponentLevelSet();

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);


    virtual void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, cv::Matx<float, 7, 1> &jacobian, cv::Matx<float, 7, 7> &hessian_approx, float &error);

    /**
    * Load the shaders we use to compute the projections and contours for the pose estimation.
    */
    void LoadShaders();

    /**
    * Construct a signed distance function and the intersection images which hold the 3D points in camera space which each point on the image plane projects to. Points outside the contour project to GL_FAR.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model used for the projection.
    * @param[out] front_intersection_image A 32 bit 3 channel image of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] front_intersection_image A 32 bit 3 channel image of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    */
    virtual void ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image, cv::Mat &front_normal_image);

    /**
    * Render the mesh in the current pose getting the depth of the each pixel and the outer contour.
    * @param[in] mesh The model which will be projected to the image plane.
    * @param[in] camera The camera model to use for the projection.
    * @param[out] front_depth A 32 bit single channel image of the depth of the first 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] back_depth A 32 bit single channel image of the depth of the last 3D point that a ray cast from a pixel inside the contour projects to on the model (in camera coordinates).
    * @param[out] contour An 8 bit single channel image which is 0 at every point that is not on the outer contour of the projected mesh and 255 where it is.
    * @param[out] component_map An 8 bit single channel image which contains the contours from the internal color segmentation.
    */
    void RenderModelForDepthAndContour(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_depth, cv::Mat &back_depth);

    float GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_probability, const size_t neighbour_probability);


    /**
    * Get the error value for the current image.
    * @param[in] classification_image The classification image for this frame.
    * @param[in] row_idx The x-pixel coordinate for this image.
    * @param[in] col_idx The y-pixel coordinate for this image.
    * @param[in] sdf_value The signed distance function value for this pixel.
    * @param[in] target_label The target label for multiclass classification.
    * @param[in] fg_area The area of the foreground region.
    * @param[in] bg_area The area of the background region.
    * @return The error value.
    */
    float GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const size_t target_probability, const size_t neighbour_probability) const;


  protected:

    unsigned char ComputeNearestNeighbourForPixel(int r, int c, float sdf_value, const int width, const int height);

    ci::gl::Fbo component_map_framebuffer_;

    ci::gl::GlslProg component_shader_;

    struct HomogenousComponent {

      HomogenousComponent(size_t tp) : target_probability(tp) {}
      cv::Mat sdf_image;
      size_t target_probability;
      cv::Mat binary_image;
      cv::Mat contour_image;

    };

    cv::Mat flow_frame;
    cv::Mat previous_frame;

    cv::Mat component_map_;
    std::vector<HomogenousComponent> components_;

    cv::Ptr<cv::DenseOpticalFlow> optical_flow;

  };


}

#endif