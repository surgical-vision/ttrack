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

struct CalibParams {

  CalibParams() : FX(0), FY(0), PX(0), PY(0), stereo_camera_translation(ci::Vec3f(0, 0, 0)), stereo_camera_rotation(ci::Vec4f(1, 0, 0, 0)) {}
  CalibParams(const float fx, const float fy, const float px, const float py) : FX(fx), FY(fy), PX(px), PY(py), stereo_camera_translation(ci::Vec3f(0,0,0)), stereo_camera_rotation(ci::Vec4f(1,0,0,0)) { }
  CalibParams(const float fx, const float fy, const float px, const float py, const ci::Vec3f &stereo_camera_translation, const ci::Vec4f &stereo_camera_rotation) : FX(fx), FY(fy), PX(px), PY(py), stereo_camera_translation(stereo_camera_translation), stereo_camera_rotation(stereo_camera_rotation) { }
  
  float FX;
  float FY;
  float PX;
  float PY;

  ci::Vec3f stereo_camera_translation; //zero vector for left/monocular camera
  ci::Vec4f stereo_camera_rotation; //identity for left/monocular camera

};

class TTrackApp : public AppBasic {

public:

  void setup();
  void update();
  void draw();

  void keyDown( KeyEvent event);
  void mouseMove( MouseEvent event );
  void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void resize();

protected:

  void drawMeshes();
  void drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &background, boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose, const CalibParams &params);
  void drawModelAtPose(boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose);
  bool returnRenderable();
  void drawRenderable(boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose);
  void setGlProjectionFromCameraCalibration(const CalibParams &params);
  void checkRenderer();
  void convertZBufferToDepth(cv::Mat &zbuffer) const;
  void setupEye(const CalibParams &params);

  // shader and texture for our model
  gl::GlslProg shader_;
  gl::Texture	left_frame_texture_;
  gl::Texture right_frame_texture_;
  gl::Texture	model_texture_;

  boost::shared_ptr<gl::Fbo> window_framebuffer_;
  boost::shared_ptr<gl::Fbo> left_external_framebuffer_;
  boost::shared_ptr<gl::Fbo> right_external_framebuffer_;
  std::unique_ptr<ttrk::Renderable> to_render_;
  
  CalibParams left_params_;
  CalibParams right_params_;

  // our camera
  MayaCamUI	maya_cam_;

  // keep track of the mouse
  Vec2i	mouse_pos_;

  ttrk::TTrack::ImageRenderSet irs_;

};

#endif
