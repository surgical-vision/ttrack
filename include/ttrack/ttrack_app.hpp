#ifndef __TTRACK_APP_HPP__
#define __TTRACK_APP_HPP__
#include <cinder/app/AppBasic.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/ImageIo.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/gl/Fbo.h>
#include <cinder/params/Params.h>

#include "ttrack.hpp"
#include "utils/camera.hpp"
#include "utils/sub_window.hpp"

using namespace ci;
using namespace ci::app;

/**
* @class TTrackApp
* @brief The GUI frontend for the project.
*
* This is the window handler, drawing the current pose estimates and handling mouse/keyboard interaction.
*/

class TTrackApp : public AppBasic {

public:

  enum ExtraViewMode { EXTRA_VIEW_PLOTTER, EXTRA_VIEW_3D, EXTRA_VIEW_DETECTOR, EXTRA_VIEW_LOCALIZER };

  /**
  * Initialization code.
  */
  virtual void setup();

  /**
  * Get the latest pose updates.
  */
  virtual void update();

  /**
  * Draw the latest pose estimates.
  */
  virtual void draw();

  /**
  * Shutdown the system, saving the results.
  */
  virtual void shutdown(); 

  /**
  * Drop a config file.
  */
  virtual void fileDrop( FileDropEvent f_event);
  
  /**
  * Capture keyboard events.
  */
  virtual void keyDown( KeyEvent k_event);

  /**
  * Capture mouse movement events.
  */
  virtual void mouseMove( MouseEvent m_event );

  /**
  * Capture mouse click events.
  */
  virtual void mouseDown( MouseEvent m_event );

  /**
  * Capture mouse drag events.
  */
  virtual void mouseDrag( MouseEvent m_event );

  /**
  * Capture window resizes.
  */
  virtual void resize();

  void show3DScene() { show_extra_view_ = EXTRA_VIEW_3D; }
  void showPlotter() { show_extra_view_ = EXTRA_VIEW_PLOTTER; }
  void showDetectorOutput() { show_extra_view_ = EXTRA_VIEW_DETECTOR; }
  void showLocalizerOutput() { show_extra_view_ = EXTRA_VIEW_LOCALIZER; }

  void reset3DViewerPosition() { reset_3D_viewport_ = true; }

protected:

  void drawImageOnCamera(const gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br);
  void drawCamera(const gl::Texture &image_data);

  /**
  * Load the helper window.
  * @param[in] window The SubWindow to draw on.
  */
  void drawHelpWindow(ttrk::SubWindow &window); 

  /**
  * Save the results from all of the pose estimates for the current frame.
  */
  void saveResults();

  /**
  * Draw a toolbar to contain buttons etc.
  */
  void drawToolbar();

  /**
  * Draw a grid, useful for drawing behind the model to give perspective.
  * @param[in] size The size of the grid.
  * @param[in] step The size of each grid square.
  * @param[in] plane_position The z coordinate of the plane.
  */
  void drawGrid(float size = 100.0f, float step = 1.0f, float plane_position = 0.0f);

  /**
  * Draw the frame error plot.
  * @param[in] window The window to draw in.
  */
  void drawPlotter(ttrk::SubWindow &window);

  /**
  * Draw 3D scene view.
  * @param[in] window The SubWindow to draw on.
  * @param[in] camera_view The view that the camera sees, renders onto the camera wireframe model for realism.
  * @param[in] cam The camera we are drawing in the scene.
  */
  void draw3D(ttrk::SubWindow &window, const gl::Texture &camera_view, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  /**
  * Draw the view of the tracked objects from one eye.
  * @param[in] window The SubWindow to draw on.
  * @param[in] background The background to draw behind the rendered objects.
  * @param[in] cam The camera view to draw from.
  */
  void drawEye(ttrk::SubWindow &window, gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam, bool draw_models = true);

  /**
  * Setup tracking from a configuration file.
  * @param[in] path The filesystem path of the configuration file.
  */
  void SetupFromConfig(const std::string &path);
  
  /**
  * Helper function to draw the model at some pose in front of the camera.
  * @param[in] window The SubWindow to draw on.
  * @param[in] mesh The model to draw.
  * @param[in] cam The camera to draw in front of.
  */
  void drawModelOnEye(ttrk::SubWindow &window, const boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam);
  
  /**
  * Render the background on the camera view.
  * @param[in] background The background texture.
  * @param[in] target_size The size of framebuffer we're drawing on 
  */
  void drawBackground(gl::Texture &background, const cv::Size target_size);
  void drawImageContents(ttrk::SubWindow &window, gl::Texture &image);

  /**
  * Render the background on a subwindow.
  * @param[in] window The SubWindow to draw on.
  * @param[in] background The background texture.
  */
  void drawBackground(ttrk::SubWindow &window, ci::gl::Texture &background);

  /**
  * Draw the additional info window contents
  */
  void drawExtra();

  void startTracking();

  bool reset_3D_viewport_; /**< Reset the position of the camera in the 3D viewport window in case it gets lost */

  ExtraViewMode show_extra_view_; /**< Type of contents for the extra view. */

  gl::GlslProg shader_; /**< The shader to do the rendering. */
   
  boost::shared_ptr<ttrk::StereoCamera> camera_; /**< The camera model we are using. */
  
  std::vector<boost::shared_ptr<ttrk::Model> > models_to_draw_; /**< The set of models we are rendering/tracking. */

  bool force_new_frame_; /**< Force the gradient descent to stop converging and for a new frame to load. */

  bool run_tracking_; /**< Switch on and off the tracking. */

  MayaCamUI	maya_cam_; /**< Maya cam for interactive viewing. */

  Vec2i	mouse_pos_; /**< Current estimate of mouse position. */

  size_t width_; /**< The width of the main window. */
  size_t height_; /**< The height of the main window. */

  ttrk::SubWindow toolbar_; /**< The toolbar subwindow. */
  std::vector<ttrk::SubWindow> windows_; /**< The main visualization sub windows. */

  ci::gl::Texture left_eye_image_; /**< The toolbar subwindow. */
  ci::gl::Texture right_eye_image_; /**< The toolbar subwindow. */
  ci::gl::Texture left_detector_image_; /**< The toolbar subwindow. */
  ci::gl::Texture right_detector_image_; /**< The current output of the right detector. */
  ci::gl::Texture localizer_image_; /**< The toolbar subwindow. */


};

#endif
