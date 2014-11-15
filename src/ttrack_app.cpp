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
#include "../include/utils/config_reader.hpp"

#include <ceres/ceres.h>

using namespace ci;
using namespace ci::app;

namespace {

  const ci::Vec3f L2R_TRANS(-6.1282f, -1.0395f, -0.5295f); //for T = [ 6.1282 ;-1.0395 ; - 0.5295 ] transformation is inversed but y,z need to be flipped as usual
  const ci::Vec4f L2R_ROT(0.3408f, -0.9392f, 0.0411f, 0.1324f); //[axis, angle] from get_angle_axis(rodrigues(inv(R)) where get_angle_axis has n_mat = flip * r_mat * flip; 

}


void TTrackApp::SetupFromConfig(const std::string &path){

  ttrk::ConfigReader reader(path);

  const std::string root_dir = reader.get_element("root-dir");

  if (!boost::filesystem::is_directory(reader.get_element("root-dir")))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(reader.get_element("output-dir"))){
    boost::filesystem::create_directory(reader.get_element("output-dir"));
  }

  auto &ttrack = ttrk::TTrack::Instance();
  ttrack.SetUp(root_dir + "/" + reader.get_element("trackable"), root_dir + "/" + reader.get_element("camera-config"), root_dir + "/" + reader.get_element("classifier-config"), reader.get_element("output-dir"), ttrk::TTrack::ClassifierFromString(reader.get_element("classifier-type")), root_dir + "/" + reader.get_element("left-input-video"), root_dir + "/" + reader.get_element("right-input-video"), ttrk::TTrack::PoseFromString(reader.get_element("starting-pose")));

  camera_.reset(new ttrk::StereoCamera(root_dir + "/" + reader.get_element("camera-config")));

  setWindowSize(2*camera_->left_eye()->Width(), camera_->left_eye()->Height());

  left_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  right_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));

}

void TTrackApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  google::InitGoogleLogging(cmd_line_args[0].c_str());
  google::SetLogDestination(google::GLOG_INFO, "z:/file0.txt");

  if (cmd_line_args.size() == 2){

    try{

      SetupFromConfig(cmd_line_args[1]);

    }
    catch (std::runtime_error){

      ci::app::console() << "Error, input file is bad!\n";

    }

  }
  else{

    const int default_width = 600, default_height = 500;
    setWindowSize(2*default_width, default_height);
    
    left_external_framebuffer_.reset(new gl::Fbo(default_width, default_height));
    right_external_framebuffer_.reset(new gl::Fbo(default_width, default_height));

  }
  
  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));


}

void TTrackApp::update(){

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  models_to_draw_.clear();
  ttrack.GetUpdate(models_to_draw_);

  boost::shared_ptr<const sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<const sv::StereoFrame>(ttrack.GetPtrToCurrentFrame());

  left_frame_texture_ = ci::fromOcv(stereo_frame->GetLeftImage());
  right_frame_texture_ = ci::fromOcv(stereo_frame->GetRightImage());

}

void TTrackApp::shutdown(){

  //_CrtMemDumpAllObjectsSince(0x0);
  //_CrtDumpMemoryLeaks();
  //system("pause");
  
  ttrk::TTrack::Destroy();

  cinder::app::AppNative::shutdown();

}

void TTrackApp::drawBackground(gl::Texture &background){

  if (!background || background.getWidth() == 0 || background.getHeight() == 0) return;

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glViewport(0, 0, background.getWidth(), background.getHeight());
  glScissor(0, 0, background.getWidth(), background.getHeight());

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, background.getWidth(), 0, background.getHeight(), 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);

  background.setFlipped(true);

  gl::draw(background);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glViewport(vp[0], vp[1], vp[2], vp[3]);

}

void TTrackApp::drawEye(boost::shared_ptr<ci::gl::Fbo> framebuffer, ci::gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam){

  framebuffer->bindFramebuffer();

  ci::gl::clear(Color(0, 0, 0));

  ci::gl::disableDepthRead();

  drawBackground(background);

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
  ci::gl::pushMatrices();
    
  cam->SetupCameraForDrawing();

  //shader_.bind();
  //shader_.uniform("tex0", 0);

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    models_to_draw_[i]->Render(false);
  }

  //shader_.unbind();

  cam->ShutDownCameraAfterDrawing();

  ci::gl::popMatrices();

  framebuffer->unbindFramebuffer();
}


void TTrackApp::draw(){

  gl::clear(ci::Color(0, 0, 0), true);

  auto &ttrack = ttrk::TTrack::Instance();
  if (ttrack.IsRunning()){

    drawEye(left_external_framebuffer_, left_frame_texture_, camera_->left_eye());
    drawEye(right_external_framebuffer_, right_frame_texture_, camera_->right_eye());

  }

  glViewport(0, 0, 2 * camera_->left_eye()->Width(), camera_->left_eye()->Height());

  gl::draw(left_external_framebuffer_->getTexture(), ci::Rectf(0.0f, (float)left_external_framebuffer_->getHeight(), (float)left_external_framebuffer_->getWidth(), 0.0f));
  gl::draw(right_external_framebuffer_->getTexture(), ci::Rectf((float)right_external_framebuffer_->getWidth(), (float)right_external_framebuffer_->getHeight(), 2.0f * (float)right_external_framebuffer_->getWidth(), 0.0f));


}

void TTrackApp::fileDrop(FileDropEvent f_event){

  try{
    for (size_t i = 0; i < f_event.getNumFiles(); ++i)
      SetupFromConfig(f_event.getFile(i).string());
  }
  catch (std::runtime_error){
    ci::app::console() << "Error loading from this config file\n";
  }

}

void TTrackApp::keyDown(KeyEvent k_event){

  if (k_event.getChar() == ' '){

    quit();

  } 
  
}

void TTrackApp::mouseMove(MouseEvent m_event){
  // keep track of the mouse
  mouse_pos_ = m_event.getPos();
}

void TTrackApp::mouseDown(MouseEvent m_event){
  // let the camera handle the interaction
  maya_cam_.mouseDown(m_event.getPos());
}

void TTrackApp::mouseDrag(MouseEvent m_event){
  // keep track of the mouse
  mouse_pos_ = m_event.getPos();

  // let the camera handle the interaction
  maya_cam_.mouseDrag(m_event.getPos(), m_event.isLeftDown(), m_event.isMiddleDown(), m_event.isRightDown());
}

void TTrackApp::resize(){
  CameraPersp cam = maya_cam_.getCamera();
  cam.setAspectRatio( getWindowAspectRatio() );
  maya_cam_.setCurrentCam( cam );
}

CINDER_APP_NATIVE( TTrackApp, RendererGl )

