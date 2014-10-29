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

class TTrackApp : public AppBasic {

public:

  void setup();
  void update();
  void draw();
  virtual void shutdown(); 

  void keyDown( KeyEvent event);
  void mouseMove( MouseEvent event );
  void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void resize();

protected:

  void drawMeshes();
  void drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &background, const boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam);
  void drawRenderable(boost::shared_ptr<ttrk::Model> mesh);
  //void setGlProjectionFromCameraCalibration(const CalibParams &params);
  //void setupEye(const CalibParams &params);

  // shader and texture for our model
  gl::GlslProg shader_;
  gl::Texture	left_frame_texture_;
  gl::Texture right_frame_texture_;
  gl::Texture	model_texture_;

  boost::shared_ptr<gl::Fbo> window_framebuffer_;
  boost::shared_ptr<gl::Fbo> left_external_framebuffer_;
  boost::shared_ptr<gl::Fbo> right_external_framebuffer_;
  
  boost::shared_ptr<ttrk::StereoCamera> camera_;
  std::vector<boost::shared_ptr<ttrk::Model> > models_to_draw_; 

  // our camera
  MayaCamUI	maya_cam_;

  // keep track of the mouse
  Vec2i	mouse_pos_;

};

#endif
