#include "../include/ttrack_app.hpp"
#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/TriMesh.h"
#include "../include/resources.hpp"
#include "cinder/ObjLoader.h"
#include "cinder/Json.h"
#include <vector>
#include <utility>
#include <boost/ref.hpp>
#include "CinderOpenCV.h"
#include <cinder/gl/Fbo.h>
using namespace ci;
using namespace ci::app;

namespace {

  bool STEREO = true;

  const std::string ROOT_DIR("../data/lnd");
  const std::size_t WINDOW_HEIGHT = 288;
  const std::size_t WINDOW_WIDTH = 736;

  const float LEFT_FX = 848.7527;
  const float LEFT_FY = 457.6367;
  const float LEFT_PX = 361.7452;
  const float LEFT_PY = WINDOW_HEIGHT - 175.8229;

  const float RIGHT_FX = 840.0076;
  const float RIGHT_FY = 452.5958;
  const float RIGHT_PX = 361.9466;
  const float RIGHT_PY = WINDOW_HEIGHT - 158.4793;

  const ci::Vec3f L2R_TRANS(-6.1282, -1.0395, -0.5295); //for T = [ 6.1282 ;-1.0395 ; - 0.5295 ] transformation is inversed but y,z need to be flipped as usual
  const ci::Vec4f L2R_ROT(0.3408, -0.9392, 0.0411, 0.1324); //[axis, angle] from get_angle_axis(rodrigues(inv(R)) where get_angle_axis has n_mat = flip * r_mat * flip; 
}




void TTrackApp::setup(){

  const std::string root_dir = ROOT_DIR;

  auto &ttrack = ttrk::TTrack::Instance();
  ttrack.SetUp(root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF, ttrk::STEREO, root_dir + "/left.avi", root_dir + "/right.avi");
  
  shader_ = gl::GlslProg( loadResource( RES_SHADER_VERT ), loadResource( RES_SHADER_FRAG ) );
 
  setWindowSize(WINDOW_WIDTH * 2, WINDOW_HEIGHT);

  CameraPersp cam;
  cam.setEyePoint(ci::Vec3f(0, 0, 0));
  cam.setViewDirection(ci::Vec3f(0, 0, -1));
  cam.setWorldUp(ci::Vec3f(0, 1, 0));
  maya_cam_.setCurrentCam(cam);

  left_params_ = CalibParams(::LEFT_FX, ::LEFT_FY, ::LEFT_PX, ::LEFT_PY);
  right_params_ = CalibParams(::RIGHT_FX, ::RIGHT_FY, ::RIGHT_PX, ::RIGHT_PY);

  gl::Fbo::Format msaaFormat;
  msaaFormat.setSamples(4); // enable 4x MSAA

  window_framebuffer_.reset(new gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT));
  left_external_framebuffer_.reset(new gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT));
  right_external_framebuffer_.reset(new gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT));
  boost::thread main_thread(boost::ref(ttrack));
 
}

void TTrackApp::update(){

  returnRenderable(); //check to see if the renderer has processed any frames

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.GetLatestUpdate(irs_)){
    return;
  }
  
  //load the 'background' frames from ttrack
  ci::ImageSourceRef img;
  if (::STEREO){
    boost::shared_ptr<sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<sv::StereoFrame>(irs_->first);
    img = ci::fromOcv(stereo_frame->GetLeftImage());
    left_frame_texture_ = gl::Texture(img);
    img = ci::fromOcv(stereo_frame->GetRightImage());
    right_frame_texture_ = gl::Texture(img);
  }
  else{
    img = ci::fromOcv(irs_->first->GetImageROI());
    left_frame_texture_ = gl::Texture(img);
  }

}

void TTrackApp::convertZBufferToDepth(cv::Mat &zbuffer) const {
 

  const float &znear = maya_cam_.getCamera().getNearClip();
  const float &zfar = maya_cam_.getCamera().getFarClip();

  for (int r = 0; r < zbuffer.rows; ++r){
    for (int c = 0; c < zbuffer.cols; ++c){
      float &z_b = zbuffer.at<float>(r, c);
      const float z_n = 2.0 * z_b - 1.0;
      z_b = 2.0 * znear * zfar / (zfar + znear - z_n * (zfar - znear));
    }
  }
  

}

bool TTrackApp::returnRenderable(){

  ttrk::WriteLock w_lock(ttrk::Renderer::mutex);

  if (to_render_ == nullptr)
    return false;

  ttrk::Renderer &r = ttrk::Renderer::Instance();
 
  to_render_->canvas_ = toOcv(left_external_framebuffer_->getTexture());
  to_render_->z_buffer_ = toOcv(left_external_framebuffer_->getDepthTexture());
  convertZBufferToDepth(to_render_->z_buffer_);
  to_render_->binary_ = maya_cam_.getCamera().getFarClip() != to_render_->z_buffer_;

  if (::STEREO){
    to_render_->right_canvas_ = toOcv(right_external_framebuffer_->getTexture());
    to_render_->right_z_buffer_ = toOcv(right_external_framebuffer_->getDepthTexture());
    convertZBufferToDepth(to_render_->right_z_buffer_);
    to_render_->right_binary_ = maya_cam_.getCamera().getFarClip() != to_render_->right_z_buffer_;
  }

  r.rendered = std::move(to_render_); //give it back
  
  return true;

}

void TTrackApp::setGlProjectionFromCameraCalibration(const CalibParams &params){

  ci::Matrix44f persp;
  persp.setToNull();
  persp.m00 = params.FX;
  persp.m11 = params.FY;
  persp.m02 = -params.PX;
  persp.m12 = -params.PY;
  persp.m22 = maya_cam_.getCamera().getNearClip() + maya_cam_.getCamera().getFarClip();
  persp.m23 = maya_cam_.getCamera().getNearClip() * maya_cam_.getCamera().getFarClip();
  persp.m32 = -1;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, maya_cam_.getCamera().getNearClip(), maya_cam_.getCamera().getFarClip());
  glMultMatrixf(persp.m);

}

void TTrackApp::setupEye(const CalibParams &params){

  CameraPersp cam;
  cam.setEyePoint(params.stereo_camera_translation);
  ci::Vec3f view_dir(0, 0, -1);
  ci::Vec3f up_dir(0, 1, 0);
  view_dir = ci::Quatf(params.stereo_camera_rotation.xyz(), params.stereo_camera_rotation.w) * view_dir;
  up_dir = ci::Quatf(params.stereo_camera_rotation.xyz(), params.stereo_camera_rotation.w) * up_dir;
  cam.setViewDirection(view_dir);
  cam.setWorldUp(up_dir);
  maya_cam_.setCurrentCam(cam);

  gl::setMatrices(maya_cam_.getCamera());

  setGlProjectionFromCameraCalibration(params);

}

void TTrackApp::drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &background, boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose, const CalibParams &params){

  framebuffer->bindFramebuffer();

  gl::clear(Color(0, 0, 0), true);
  if (background.getHeight() + background.getWidth() > 0)
    gl::draw(background);
  
  framebuffer->unbindFramebuffer();
  ci::writeImage("../../fbuffer_image.png", framebuffer->getTexture());
  return;

  GLint viewport_cache[4];
  glGetIntegerv(GL_VIEWPORT, viewport_cache);
  glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

  gl::pushMatrices();

  setupEye(params);

  gl::multModelView(pose.AsCiMatrixForOpenGL());
  
  drawModelAtPose(mesh, pose);

  glViewport(viewport_cache[0], viewport_cache[1], viewport_cache[2], viewport_cache[3]);

  gl::popMatrices();

  framebuffer->unbindFramebuffer();

}

void TTrackApp::checkRenderer(){

  ttrk::WriteLock w_lock(ttrk::Renderer::mutex);
  
  ttrk::Renderer &r = ttrk::Renderer::Instance();

  if (r.to_render.get() != nullptr && r.rendered.get() == nullptr){
    to_render_ = std::move(r.to_render); //take ownership
    //drawRenderable(to_render_->mesh_,to_render_->pose_); //make the draw command 
    drawModelOnEye(left_external_framebuffer_, gl::Texture(0, 0), to_render_->mesh_, to_render_->pose_, left_params_);

    if (::STEREO){


      drawModelOnEye(right_external_framebuffer_, gl::Texture(0, 0), to_render_->mesh_, to_render_->pose_, right_params_);
      

    }

  }

}

void TTrackApp::drawModelAtPose(boost::shared_ptr<ttrk::Model> mesh, const ttrk::Pose &pose){
  
  auto meshes_textures_transforms = mesh->GetRenderableMeshes();
  ci::Matrix44d current_pose = pose.AsCiMatrixForOpenGL();

  for (auto mesh_tex_trans = meshes_textures_transforms.begin(); mesh_tex_trans != meshes_textures_transforms.end(); ++mesh_tex_trans){

    auto texture = mesh_tex_trans->get<1>();

    gl::pushModelView();
    gl::multModelView(current_pose * mesh_tex_trans->get<2>());

    const auto trimesh = mesh_tex_trans->get<0>();
    gl::draw(*trimesh);

    gl::popModelView();


  }
}

void TTrackApp::setupDraw() {


  gl::pushMatrices();
  gl::setMatrices(maya_cam_.getCamera());

  gl::enableDepthRead();
  gl::enableDepthWrite();

  gl::enableAlphaBlending();

  shader_.bind();

}

void TTrackApp::finishDraw() {

  shader_.unbind();
  gl::disableAlphaBlending();

  gl::popMatrices();

}

void TTrackApp::drawMonocular(){

  drawModelOnEye(window_framebuffer_, left_frame_texture_, irs_->second[0].PtrToModel(), irs_->second[0].CurrentPose(), left_params_);
  gl::draw(window_framebuffer_->getTexture());

}

void TTrackApp::drawStereo(){

  drawModelOnEye(window_framebuffer_, left_frame_texture_, irs_->second[0].PtrToModel(), irs_->second[0].CurrentPose(), left_params_);
  gl::draw(window_framebuffer_->getTexture(), ci::Rectf(0, left_frame_texture_.getHeight(), left_frame_texture_.getWidth(), 0));

  drawModelOnEye(window_framebuffer_, right_frame_texture_, irs_->second[0].PtrToModel(), irs_->second[0].CurrentPose(), right_params_);
  gl::draw(window_framebuffer_->getTexture(), ci::Rectf(window_framebuffer_->getWidth(), window_framebuffer_->getHeight(), 2 * window_framebuffer_->getWidth(), 0));

}


void TTrackApp::draw(){
  
  setupDraw();
  
  checkRenderer();
  
  finishDraw();

  if (!irs_){
    return;
  }

  setupDraw();

  if (::STEREO)
    drawStereo();
  else
    drawMonocular();
  
  finishDraw();

}


void TTrackApp::keyDown( KeyEvent event ){

  if(event.getChar() == ' '){

    quit();

  } 
  
}

void TTrackApp::mouseMove( MouseEvent event ){
  // keep track of the mouse
  mouse_pos_ = event.getPos();
}

void TTrackApp::mouseDown( MouseEvent event ){	
  // let the camera handle the interaction
  maya_cam_.mouseDown( event.getPos() );
}

void TTrackApp::mouseDrag( MouseEvent event ){
  // keep track of the mouse
  mouse_pos_ = event.getPos();

  // let the camera handle the interaction
  maya_cam_.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void TTrackApp::resize(){
  CameraPersp cam = maya_cam_.getCamera();
  cam.setAspectRatio( getWindowAspectRatio() );
  maya_cam_.setCurrentCam( cam );
}



CINDER_APP_NATIVE( TTrackApp, RendererGl )
