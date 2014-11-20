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
#include "../include/utils/plotter.hpp"

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

  windows_[0] = SubWindow(toolbar_.window_coords_.getWidth(), 0, camera_->left_eye()->Width(), camera_->left_eye()->Height());
  windows_[1] = SubWindow(toolbar_.window_coords_.getWidth() + camera_->left_eye()->Width(), 0, camera_->left_eye()->Width(), camera_->left_eye()->Height());

  windows_[2] = SubWindow(toolbar_.window_coords_.getWidth(), 
                          camera_->left_eye()->Height(), 
                          windows_[2].window_coords_.getWidth(),
                          windows_[2].window_coords_.getHeight());

  windows_[3] = SubWindow(toolbar_.window_coords_.getWidth() + windows_[2].window_coords_.getWidth(),
                          camera_->left_eye()->Height(), 
                          windows_[3].window_coords_.getWidth(), 
                          windows_[3].window_coords_.getHeight());


  //left_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  //right_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));

  resize();

}

void TTrackApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  google::InitGoogleLogging(cmd_line_args[0].c_str());
  google::SetLogDestination(google::GLOG_INFO, "z:/file0.txt");

  toolbar_ = SubWindow(0, 0, 300, 700);

  //create left eye window
  windows_.push_back(SubWindow(100, 0, 600, 500));
  windows_.push_back(SubWindow(700, 0, 600, 500));

  //create error plot
  windows_.push_back(SubWindow(100, 500, 600, 300));

  //create 3D viewer
  windows_.push_back(SubWindow(700, 500, 600, 300));

  if (cmd_line_args.size() == 2){

    try{

      SetupFromConfig(cmd_line_args[1]);

    }
    catch (std::runtime_error){

      ci::app::console() << "Error, input file is bad!\n";

    }

  }
  else{

    resize();

    //const int default_width = 600, default_height = 500;
    //setWindowSize(2*default_width, default_height);
    
    //left_external_framebuffer_.reset(new gl::Fbo(default_width, default_height));
    //right_external_framebuffer_.reset(new gl::Fbo(default_width, default_height));

  }
  
  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));
  
}

void TTrackApp::update(){

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  models_to_draw_.clear();
  ttrack.GetUpdate(models_to_draw_);

  boost::shared_ptr<const sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<const sv::StereoFrame>(ttrack.GetPtrToCurrentFrame());

  windows_[0].texture_ = ci::fromOcv(stereo_frame->GetLeftImage());
  windows_[1].texture_ = ci::fromOcv(stereo_frame->GetRightImage());

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

  drawToolbar();

  auto &ttrack = ttrk::TTrack::Instance();
  if (ttrack.IsRunning()){

    drawEye(windows_[0].framebuffer_, windows_[0].texture_, camera_->left_eye());
    drawEye(windows_[1].framebuffer_, windows_[1].texture_, camera_->right_eye());

    drawPlotter(windows_[2].framebuffer_);
    draw3D(windows_[3].framebuffer_, windows_[0].framebuffer_->getTexture(), camera_->left_eye());

  }

  glViewport(0, 0, width_, height_);

  gl::draw(toolbar_.framebuffer_->getTexture(), ci::Rectf(toolbar_.window_coords_.x1, toolbar_.window_coords_.y2, toolbar_.window_coords_.x2, toolbar_.window_coords_.y1));

  for (auto i = 0; i < windows_.size(); ++i){
  
    gl::draw(windows_[i].framebuffer_->getTexture(), ci::Rectf(windows_[i].window_coords_.x1, windows_[i].window_coords_.y2, windows_[i].window_coords_.x2, windows_[i].window_coords_.y1));
    
  }

}

void TTrackApp::drawToolbar() {

  toolbar_.framebuffer_->bindFramebuffer();

  //glViewport(0, 0, toolbar_.framebuffer_->getWidth(), toolbar_.framebuffer_->getHeight());
  //glScissor(0, 0, toolbar_.framebuffer_->getWidth(), toolbar_.framebuffer_->getHeight());
  glViewport(0, 0, width_, height_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //glMatrixMode(GL_PROJECTION);
  //glPushMatrix();
  //glLoadIdentity();
  //glOrtho(0, toolbar_.framebuffer_->getWidth(), 0, toolbar_.framebuffer_->getHeight(), -1, 1);


  gl::color(1, 1, 1);

  const int width = toolbar_.framebuffer_->getWidth();
  const int height = toolbar_.framebuffer_->getHeight();
  const int buffer = 20;

  //draw line from 20 - 20 to
  
  gl::drawLine(ci::Vec2f(buffer, buffer), ci::Vec2f(width - 2*buffer, buffer));
  gl::drawLine(ci::Vec2f(width - 2*buffer, buffer), ci::Vec2f(width - 2*buffer, height - buffer));
  gl::drawLine(ci::Vec2f(width - 2*buffer, height - buffer), ci::Vec2f(buffer, height - buffer));
  gl::drawLine(ci::Vec2f(buffer, height - buffer), ci::Vec2f(buffer, buffer));

  toolbar_.framebuffer_->unbindFramebuffer();

}

void TTrackApp::drawPlotter(boost::shared_ptr<gl::Fbo> framebuffer) {

  framebuffer->bindFramebuffer();

  glViewport(0, 0, framebuffer->getWidth(), framebuffer->getHeight());

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  gl::clear(ci::Color(1,1,1));

  gl::color(0, 0, 0);

  const int width = framebuffer->getWidth();
  const int height = framebuffer->getHeight();
  const int y_edge = 50;
  const int x_edge = 50;
  
  const int start_x = x_edge;
  const int end_x = x_edge + (int)(2.8 * width);
  const int start_y = y_edge + 2 * height;
  const int end_y = y_edge;

  int num_steps = 50;
  const int normalized_height = end_y - start_y;
  const int normalized_width = end_x - start_x;

  gl::drawLine(ci::Vec2f(start_x, start_y), ci::Vec2f(start_x, end_y));
  gl::drawLine(ci::Vec2f(start_x, start_y), ci::Vec2f(end_x, start_y));
  
  float largest_error = -1;
  auto &plotter = ttrk::ErrorMetricPlotter::Instance();
  auto &plots = plotter.GetPlottables();
  for (auto &plot : plots){
    auto vals = plot->GetErrorValues();
    if (vals.size() && vals.back() > largest_error){
      largest_error = vals.back();
    }
    if (vals.size() > num_steps)
      num_steps = vals.size();
  }

  if (largest_error != -1){

    const float y_range = 1.5 * largest_error;
    for (auto &plot : plots){
      auto vals = plot->GetErrorValues();
      for (auto i = 1; i < vals.size(); ++i){

        float v1 = vals[i];
        float v1_normed = v1 / y_range;
        float y1 = v1_normed * normalized_height;

        float v0 = vals[i-1];
        float v0_normed = v0 / y_range;
        float y0 = v0_normed * normalized_height;

        float x1 = (float)i / num_steps;
        x1 *= normalized_width;
        float x0 = (float)(i - 1)/num_steps;
        x0 *= normalized_width;

        gl::drawLine(ci::Vec2i(start_x + x0, start_y + y0), ci::Vec2i(start_x + x1, start_y + y1));

      }
    }

  }

  gl::color(1, 1, 1);

  framebuffer->unbindFramebuffer();

} 

void TTrackApp::draw3D(boost::shared_ptr<gl::Fbo> framebuffer, const gl::Texture &camera_view, const boost::shared_ptr<ttrk::MonocularCamera> cam) {

  framebuffer->bindFramebuffer();

  gl::clear(Color(0.0, 0.0, 0.0));

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  ci::Vec3f test_start_pose(10, -5, 60);

  ci::CameraPersp maya;
  maya.setEyePoint(ci::Vec3f(-20, -20, -20));
  maya.setWorldUp(ci::Vec3f(0, 1, 0));
  maya.lookAt(test_start_pose);
  
  gl::pushMatrices();
  gl::setMatrices(maya);

  ci::Area viewport = gl::getViewport();
  gl::setViewport(ci::Area(0, 0, framebuffer->getWidth(), framebuffer->getHeight()));

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
  ci::gl::pushMatrices();
  
  gl::pushModelView();
  //cam->SetupCameraForDrawing();
  
  cam->SetupLight();

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    models_to_draw_[i]->Render(false);
  }

  cam->ShutDownCameraAfterDrawing();

  gl::popModelView();

  ci::gl::popMatrices();

  gl::setViewport(viewport);
  gl::popMatrices();

  framebuffer->getTexture().setFlipped(true);
  framebuffer->unbindFramebuffer();

}

void TTrackApp::drawGrid(float size, float step, float plane_position){

  gl::color(Colorf(0.5f, 0.5f, 0.5f));

  float start = -size;

  for (float i = -size; i <= size; i += step){
    gl::drawLine(ci::Vec3f(start, i, plane_position), ci::Vec3f(size, i, plane_position));
    gl::drawLine(ci::Vec3f(i, start, plane_position), ci::Vec3f(i, size, plane_position));
  }

  gl::color(1.0, 1.0, 1.0);

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

  int width = toolbar_.window_coords_.getWidth();
  int height = 0;

  const int width_of_eyes = windows_[0].window_coords_.getWidth() + windows_[1].window_coords_.getWidth();
  const int width_of_viewers = windows_[2].window_coords_.getWidth() + windows_[3].window_coords_.getWidth();

  const int height_of_toolbar = toolbar_.window_coords_.getHeight();
  const int height_of_eyes_and_left_viewer = windows_[0].window_coords_.getHeight() + windows_[2].window_coords_.getHeight();
  const int height_of_eyes_and_right_viewer = windows_[0].window_coords_.getHeight() + windows_[3].window_coords_.getHeight();

  width += std::max<int>(width_of_eyes, width_of_viewers);
  height += std::max<int>(height_of_toolbar, std::max<int>(height_of_eyes_and_left_viewer, height_of_eyes_and_right_viewer));

  setWindowSize(width, height);

  width_ = width;
  height_ = height;

  CameraPersp cam = maya_cam_.getCamera();
  cam.setAspectRatio( getWindowAspectRatio() );
  maya_cam_.setCurrentCam( cam );
}

CINDER_APP_NATIVE( TTrackApp, RendererGl )

