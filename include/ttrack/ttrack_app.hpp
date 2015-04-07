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

protected:

  void drawImageOnCamera(const gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br);
  void drawCamera(const gl::Texture &image_data);

  /**
  * Load the helper window.
  * @param[in] framebuffer The framebuffer to draw the helper window in.
  */
  void drawHelpWindow(boost::shared_ptr<ci::gl::Fbo> framebuffer); 

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
  * @param[in] framebuffer The framebuffer to draw in.
  */
  void drawPlotter(boost::shared_ptr<gl::Fbo> framebuffer);

  /**
  * Draw 3D scene view.
  * @param[in] framebuffer The framebuffer to draw in.
  * @param[in] camera_view The view that the camera sees, renders onto the camera wireframe model for realism.
  * @param[in] cam The camera we are drawing in the scene.
  */
  void draw3D(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &camera_view, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  /**
  * Draw the view of the tracked objects from one eye.
  * @param[in] framebuffer The framebuffer to draw on.
  * @param[in] background The background to draw behind the rendered objects.
  * @param[in] cam The camera view to draw from.
  */
  void drawEye(boost::shared_ptr<gl::Fbo> framebuffer, gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  /**
  * Setup tracking from a configuration file.
  * @param[in] path The filesystem path of the configuration file.
  */
  void SetupFromConfig(const std::string &path);
  
  /**
  * Helper function to draw the model at some pose in front of the camera.
  * @param[in] framebuffer The framebuffer to render to.
  * @param[in] mesh The model to draw.
  * @param[in] cam The camera to draw in front of.
  */
  void drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam);
  
  /**
  * Render the background on the camera view.
  * @param[in] background The background texture.
  */
  void drawBackground(gl::Texture &background);
  
  gl::GlslProg shader_; /**< The shader to do the rendering. */
   
  boost::shared_ptr<ttrk::StereoCamera> camera_; /**< The camera model we are using. */
  std::vector<boost::shared_ptr<ttrk::Model> > models_to_draw_; /**< The set of models we are rendering/tracking. */

  bool force_new_frame_; /**< Force the gradient descent to stop converging and for a new frame to load. */

  MayaCamUI	maya_cam_; /**< Maya cam for interactive viewing. */

  Vec2i	mouse_pos_; /**< Current estimate of mouse position. */

  /**
  * @struct SubWindow
  * @brief Sub window regions to draw to in the main viewer.
  * Allows the main window to be split up into different viewports.
  */
  struct SubWindow {
    
    /**
    * Empty constructor.
    */
    SubWindow() { }

    /**
    * Create a window with dimensions.
    * @param[in] start_x The x coordinate of the top left of the box in the main window reference frame.
    * @param[in] start_y The y coordinate of the top left of the box in the main window reference frame.
    * @param[in] width The height of the sub window in pixels.
    * @param[in] height The width of the sub window in pixels.
    */
    SubWindow(int start_x, int start_y, int width, int height){
      Init(start_x, start_y, width, height, width, height);
    }

    /**
    * Create a window with dimensions.
    * @param[in] start_x The x coordinate of the top left of the box in the main window reference frame.
    * @param[in] start_y The y coordinate of the top left of the box in the main window reference frame.
    * @param[in] width The height of the sub window in pixels.
    * @param[in] height The width of the sub window in pixels.
    */
    SubWindow(int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height)  {
      
      Init(start_x, start_y, eye_width, eye_height, draw_width, draw_height);

    }

    void Init(int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height){
      window_coords_ = ci::Rectf(start_x, start_y, start_x + draw_width, start_y + draw_height);
      gl::Fbo::Format f;
      f.enableMipmapping();
      framebuffer_.reset(new gl::Fbo(eye_width, eye_height, f));
      texture_ = gl::Texture(eye_width, eye_height);
    }


    ci::Rectf window_coords_; /**< The window coordinates within the main window reference frame. */
    boost::shared_ptr<gl::Fbo> framebuffer_; /**< The framebuffer of size width, height which is rendered to when we draw to this subwindow. */
    gl::Texture texture_; /**< The texture that is attached to this framebuffer. */

  };

  size_t width_; /**< The width of the main window. */
  size_t height_; /**< The height of the main window. */

  SubWindow toolbar_; /**< The toolbar subwindow. */
  std::vector<SubWindow> windows_; /**< The main visualization sub windows. */

  ci::params::InterfaceGlRef	menubar_; /**< The widgets draw in the toolbar subwindow. */

};

#endif
