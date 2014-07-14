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
#include "utils/renderer.hpp"

using namespace ci;
using namespace ci::app;

class TTrackApp : public AppBasic {

public:

  void setup();
  void update();
  void draw();

  void drawGrid(float size=100.0f, float step=10.0f);

  void keyDown( KeyEvent event);
  void mouseMove( MouseEvent event );
  void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void resize();

protected:

  bool returnRenderable();
  void drawRenderable(boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose);
  void checkRenderer();
  void convertZBufferToDepth(cv::Mat &zbuffer) const;

  // shader and texture for our model
  gl::GlslProg shader_;
  gl::Texture	frame_texture_;
  gl::Texture	model_texture_;

  boost::shared_ptr<gl::Fbo> framebuffer_;
  std::unique_ptr<ttrk::Renderable> to_render_;
  
  void draw2D();
  void draw3D();
  void drawMeshes();

  // our camera
  MayaCamUI	maya_cam_;

  // keep track of the mouse
  Vec2i	mouse_pos_;

  // keep track of time
  double time_;

  //TTrack factory
  //boost::shared_ptr<ttrk::TTrack> t_track_;

  ttrk::TTrack::ImageRenderSet irs_;

};

#endif
