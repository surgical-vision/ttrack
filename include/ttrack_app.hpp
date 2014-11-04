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

  void drawEye(boost::shared_ptr<gl::Fbo> framebuffer, gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam);

  void SetupFromConfig(const std::string &path);
  
  void drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam);
  
  void drawBackground(gl::Texture &background);
  
  gl::GlslProg shader_;
 
  gl::Texture	left_frame_texture_;
  gl::Texture right_frame_texture_;

  boost::shared_ptr<gl::Fbo> left_external_framebuffer_;
  boost::shared_ptr<gl::Fbo> right_external_framebuffer_;
  
  boost::shared_ptr<ttrk::StereoCamera> camera_;
  std::vector<boost::shared_ptr<ttrk::Model> > models_to_draw_; 

  MayaCamUI	maya_cam_;

  Vec2i	mouse_pos_;

};

#endif
