#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__

#include <cinder/app/Renderer.h>

#include "../model/model.hpp"
#include "../../utils/image.hpp"
#include "features/register_points.hpp"
#include <ttrack/constants.hpp>

namespace ttrk {

  /**
  * @class Localizer
  * @brief An abstract class to represent pose localization algorithms.
  */
  class Localizer {

  public:

    static cv::Mat occlusion_image;

    static void ResetOcclusionImage() {

      occlusion_image = GL_FAR * cv::Mat::ones(occlusion_image.size(), CV_32FC1);

    }

    static void UpdateOcclusionImage(const cv::Mat &depth){

      for (int r = 0; r < depth.rows; ++r){
        for (int c = 0; c < depth.cols; ++c){
          if (occlusion_image.at<float>(r, c) > depth.at<cv::Vec4f>(r, c)[0]){
            occlusion_image.at<float>(r, c) = depth.at<cv::Vec4f>(r, c)[0];
          }
        }
      }
    }

    Localizer() : first_run_(true), curr_step(0), NUM_STEPS(15), point_registration_weight(0.2), articulated_point_registration_weight(0.5), use_articulated_point_derivs_(true), use_point_derivs_rotation_(true), use_point_derivs_translation_(true), use_global_roll_search_first_(true), use_global_roll_search_last_(true) {}

    /**
    * Do single frame pose estimation. This method receives a model (which may or may not have some initial estimate of pose) and tries to
    * estimate a new estimate for the object in the new frame. The update is applied to the model in this method.
    * @param[in] model The model to align to the image data.
    * @param[in] frame The current frame to perform alignment against.
    */
    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame) = 0;

    /**
    * Virtual destructor.
    */
    virtual ~Localizer() {}

    /**
    * Test for convergence.
    * @return True for converged, false otherwise.
    */
    virtual bool HasConverged() = 0;

    /**
    * Setter for frame count variable.
    * @param[in] fc The current frame count.
    */
    void SetFrameCount(int fc) { frame_count_ = fc; }

    /**
    * Accessor for the progress frame for visualization in the GUI.
    * @return The progress frame.
    */
    cv::Mat GetProgressFrame() { return progress_frame_; }

    bool IsFirstRun() { return first_run_; }

    void DoneFirstStep() { first_run_ = false; }

    void SetFeatureLocalizer(boost::shared_ptr<FeatureLocalizer> point_registration) { point_registration_ = point_registration; }

    void UpdateStepCount() { curr_step++; }

    void ResetStepCount() { curr_step = 0; }

    void SetMaximumIterations(const size_t iter) { NUM_STEPS = iter; }

    void SetPointRegistrationWeight(const float weight) { point_registration_weight = weight; }
    
    void SetArticulatedPointRegistrationWeight(const float weight) { articulated_point_registration_weight = weight; }


    void SetupPointTracker(const bool use_rotations, const bool use_translations, const bool use_articulation, const bool use_global_roll_search_first, const bool use_global_roll_search_last){
      use_articulated_point_derivs_ = use_articulation;
      use_point_derivs_rotation_ = use_rotations;
      use_point_derivs_translation_ = use_translations;
      use_global_roll_search_first_ = use_global_roll_search_first;
      use_global_roll_search_last_ = use_global_roll_search_last;
    }

  protected:

    cv::Mat progress_frame_; /**< Localizer progress frame. For viewing progress in a GUI or something similar. */

    int frame_count_; /**< Current frame count */

    bool first_run_;

    boost::shared_ptr<FeatureLocalizer> point_registration_; /**< Computes the point registration error. */

    int NUM_STEPS;  /**< Number of step for the optimization. */
    int curr_step; /**< Current step in the optimization. */

    float point_registration_weight;
    float articulated_point_registration_weight;

    bool use_articulated_point_derivs_;
    bool use_point_derivs_rotation_;
    bool use_point_derivs_translation_;
    bool use_global_roll_search_first_;
    bool use_global_roll_search_last_;


  };


}


#endif
