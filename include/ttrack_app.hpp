#ifndef __TTRACK_APP_HPP__
#define __TTRACK_APP_HPP__
#include <cinder/app/AppBasic.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/ImageIo.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/gl/Fbo.h>

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

  virtual void setup();

  virtual void update();

  virtual void draw();

  virtual void shutdown(); 

  virtual void fileDrop( FileDropEvent f_event);

  virtual void keyDown( KeyEvent k_event);

  virtual void mouseMove( MouseEvent m_event );

  virtual void mouseDown( MouseEvent m_event );

  virtual void mouseDrag( MouseEvent m_event );

  virtual void resize();

protected:

  void drawToolbar();

  void drawGrid(float size = 100.0f, float step = 1.0f, float plane_position = 0.0f);

  void drawPlotter(boost::shared_ptr<gl::Fbo> framebuffer);

  void draw3D(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &camera_view, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  void drawEye(boost::shared_ptr<gl::Fbo> framebuffer, gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  void SetupFromConfig(const std::string &path);
  
  void drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam);
  
  void drawBackground(gl::Texture &background);
  
  gl::GlslProg shader_;
   
  boost::shared_ptr<ttrk::StereoCamera> camera_;
  std::vector<boost::shared_ptr<ttrk::Model> > models_to_draw_; 

  cv::VideoWriter writer_;
  std::ofstream tracked_file_;

  bool force_new_frame_;

  MayaCamUI	maya_cam_;

  Vec2i	mouse_pos_;

  struct SubWindow {
    
    SubWindow() { }

    SubWindow(int start_x, int start_y, int width, int height) : window_coords_(start_x, start_y, start_x + width, start_y + height) {
      
      gl::Fbo::Format f;
      framebuffer_.reset(new gl::Fbo(width, height, f));
      texture_ = gl::Texture(width, height);

    }

    ci::Rectf window_coords_;
    boost::shared_ptr<gl::Fbo> framebuffer_;
    gl::Texture texture_;

  };

  size_t width_;
  size_t height_;

  SubWindow toolbar_;
  std::vector<SubWindow> windows_;

};

#endif
