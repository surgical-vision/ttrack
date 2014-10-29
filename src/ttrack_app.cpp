#include <cinder/ObjLoader.h>
#include <cinder/Json.h>
#include <vector>
#include <utility>
#include <boost/ref.hpp>
#include <CinderOpenCV.h>
#include <cinder/gl/Fbo.h>
#include <cinder/app/AppNative.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/ImageIo.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/TriMesh.h>

#include "../include/headers.hpp"
#include "../include/ttrack_app.hpp"
#include "../include/resources.hpp"

using namespace ci;
using namespace ci::app;

namespace {

  const ci::Vec3f L2R_TRANS(-6.1282f, -1.0395f, -0.5295f); //for T = [ 6.1282 ;-1.0395 ; - 0.5295 ] transformation is inversed but y,z need to be flipped as usual
  const ci::Vec4f L2R_ROT(0.3408f, -0.9392f, 0.0411f, 0.1324f); //[axis, angle] from get_angle_axis(rodrigues(inv(R)) where get_angle_axis has n_mat = flip * r_mat * flip; 

}

void TTrackApp::setup(){

  const std::string root_dir = "z:/phd/ttrack/data/lnd";

  auto &ttrack = ttrk::TTrack::Instance();
  ttrack.SetUp(root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF, ttrk::STEREO, root_dir + "/left.avi", root_dir + "/right.avi");
  
  camera_.reset(new ttrk::StereoCamera(root_dir + "/camera/config.xml"));

  shader_ = gl::GlslProg( loadResource( RES_SHADER_VERT ), loadResource( RES_SHADER_FRAG ) );
  
  setWindowSize(camera_->left_eye()->Width(), camera_->left_eye()->Height());

  window_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  left_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  right_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  
}

void TTrackApp::update(){

  auto &ttrack = ttrk::TTrack::Instance();
  ttrack.GetUpdate(models_to_draw_);

  boost::shared_ptr<const sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<const sv::StereoFrame>(ttrack.GetPtrToCurrentFrame());
  ci::ImageSourceRef img = ci::fromOcv(stereo_frame->GetLeftImage());
  left_frame_texture_ = gl::Texture(img);
  img = ci::fromOcv(stereo_frame->GetRightImage());
  right_frame_texture_ = gl::Texture(img);

}

void TTrackApp::shutdown(){

  //_CrtMemDumpAllObjectsSince(0x0);
  //_CrtDumpMemoryLeaks();
  //system("pause");
  cinder::app::AppNative::shutdown();

}


void TTrackApp::drawModelOnEye(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &background, boost::shared_ptr<ttrk::Model> mesh, const boost::shared_ptr<ttrk::MonocularCamera> cam){

  framebuffer->bindFramebuffer();

  gl::clear(Color(0, 0, 0));

  //gl::disableDepthRead();
  gl::enableDepthRead();
  gl::enableDepthWrite();

  cam->SetupCameraForDrawing(framebuffer->getWidth(), framebuffer->getHeight());

  if (background.getWidth() > 0 && background.getHeight() > 0)
    gl::draw(background);
  
  mesh->Render();

  framebuffer->unbindFramebuffer();
}

void TTrackApp::draw(){

  gl::clear(ci::Color(0, 0, 0), true);

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    drawModelOnEye(left_external_framebuffer_, left_frame_texture_, models_to_draw_[i], camera_->left_eye());
    drawModelOnEye(right_external_framebuffer_, right_frame_texture_, models_to_draw_[i], camera_->right_eye());
  }
  
  gl::draw(left_external_framebuffer_->getTexture(), ci::Rectf(0, left_external_framebuffer_->getHeight(), left_external_framebuffer_->getWidth(), 0));
  gl::draw(right_external_framebuffer_->getTexture(), ci::Rectf(right_external_framebuffer_->getWidth(), right_external_framebuffer_->getHeight(), 2 * right_external_framebuffer_->getWidth(), 0));


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

