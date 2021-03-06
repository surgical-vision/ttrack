#ifndef __COMP_LS_HPP__
#define __COMP_LS_HPP__

#include "stereo_pwp3d.hpp"

#ifdef USE_CERES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>
#endif

namespace ttrk {

  class ComponentLevelSet : public StereoPWP3D {

  public:

    /**
    * Construct a component level set tracked for tracking objects by diving them into homogenous regions rather than one bag-of-pixels model.
    * @param[in] number_of_components The number of homogenous components which consistute the model. Right now this is manually estimated. Potentially we can estimate this automatically.
    * @param[in] camera The pinhole camera model to use.
    */
    ComponentLevelSet(size_t number_of_components, boost::shared_ptr<StereoCamera> camera);

    /**
    * Destructor.
    */
    virtual ~ComponentLevelSet();

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);


    virtual void ComputeJacobiansForEye(const cv::Mat &classification_image, boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, std::vector<cv::Matx<float, 7, 1> > &jacobians, std::vector<cv::Matx<float, 7, 7> > &hessian_approx, std::vector<float> &error);
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
    virtual void ProcessSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image);

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

    float GetRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_probability, const size_t neighbour_probability) const;
    float GetBinaryRegionAgreement(const cv::Mat &classification_image, const int r, const int c, const float sdf_value, const size_t target_probability, const size_t neighbour_probability) const;

    void GetRegionProbability(const cv::Mat &classification_image, const int r, const int c, const size_t target_label, const size_t neighbour_label, float &pixel_probability, float &neighbour_probability) const;

    /**
    * Get the error value for the current image.
    * @param[in] classification_image The classification image for this frame.
    * @param[in] row_idx The x-pixel coordinate for this image.
    * @param[in] col_idx The y-pixel coordinate for this image.
    * @param[in] sdf_value The signed distance function value for this pixel.
    * @param[in] target_label The target label for multiclass classification.
    * @return The error value.
    */
    float GetErrorValue(const cv::Mat &classification_image, const int row_idx, const int col_idx, const float sdf_value, const size_t target_probability, const size_t neighbour_probability, const size_t foreground_size = 1, const size_t background_size = 1) const;
    
    virtual float DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points);

#ifdef USE_CERES 

    void DoEyeCeres(double const *const *parameters, double *residuals, double **jacobians, const cv::Mat &classification_image, const boost::shared_ptr<MonocularCamera> camera) const;

    bool operator() (double const *const *parameters, double *residuals, double **jacobians) const;

    bool operator() (double const* const* parameters, double* residuals) const;

    void TrackTargetCeresOptimization();

    boost::shared_ptr<Model> current_model_;

#endif

    void ComputeScores(const cv::Mat &classification_image, float &current_score, float &best_score) const;

  protected:

    unsigned char ComputeNearestNeighbourForPixel(int r, int c, float sdf_value, const int width, const int height) const;

    ci::gl::Fbo component_map_framebuffer_; /**< The framebuffer to render the component indexing image into. */
    cv::Mat component_map_; /**< The framebuffer is vertically flipped and stored in this. */
    
    ci::gl::GlslProg component_shader_; /**< The shader which uses the model texture map to render an 'indexing' image which contains the homogenous region index. */

    struct HomogenousComponent {

      HomogenousComponent(size_t tp) : target_probability(tp) {}
      cv::Mat sdf_image; /**< Each component has it's own sdf. This means it's treated an independent bag-of-pixles model which is tracked jointly with the other components.*/
      size_t target_probability; /**< The target probability. Requires some level of coupling with the detector as the detector output must match the expected labelled of this component. */
      cv::Mat binary_image;
      cv::Mat contour_image;

    };

   
    std::vector<HomogenousComponent> components_;

  };


}

#endif