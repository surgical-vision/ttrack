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

#include "../include/ttrack/headers.hpp"
#include "../include/ttrack/ttrack_app.hpp"
#include "../include/ttrack/resources.hpp"
#include "../include/ttrack/utils/config_reader.hpp"
#include "../include/ttrack/utils/plotter.hpp"
#include "../include/ttrack/utils/UI.hpp"

using namespace ci;
using namespace ci::app;

void TTrackApp::SetupFromConfig(const std::string &path){

  ttrk::ConfigReader reader(path);

  const std::string root_dir = reader.get_element("root-dir");

  if (!boost::filesystem::is_directory(reader.get_element("root-dir")))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(reader.get_element("output-dir"))){
    boost::filesystem::create_directory(reader.get_element("output-dir"));
  }

  auto &ttrack = ttrk::TTrack::Instance();
  std::vector< std::vector <float> > starting_poses;
  for (int i = 0;; ++i){

    std::stringstream ss;
    ss << "starting-pose-" << i;
    try{
      starting_poses.push_back(ttrk::TTrack::PoseFromString(reader.get_element(ss.str())));
    }
    catch (std::runtime_error &){
      break;
    }

  }

  //load the number of labels from the config file. This should include background! 
  size_t number_of_labels = 2;
  try{
    number_of_labels = reader.get_element_as_type<size_t>("num-labels");
  }
  catch (std::runtime_error &){ }

  //throwing errors here? did you remember the zero at element 15 of start pose or alteranatively set the trackable dir to absolute in the cfg file
  ttrack.SetUp(reader.get_element("trackable"),
               root_dir + "/" + reader.get_element("camera-config"),
               root_dir + "/" + reader.get_element("classifier-config"),
               reader.get_element("output-dir"),
               ttrk::TTrack::LocalizerTypeFromString(reader.get_element("localizer-type")),
               ttrk::TTrack::ClassifierFromString(reader.get_element("classifier-type")),
               root_dir + "/" + reader.get_element("left-input-video"),
               root_dir + "/" + reader.get_element("right-input-video"),
               starting_poses,
               number_of_labels);
  
  int width = reader.get_element_as_type<int>("window-width");
  int height = reader.get_element_as_type<int>("window-height");

  camera_.reset(new ttrk::StereoCamera(root_dir + "/" + reader.get_element("camera-config")));
  
  windows_[0] = SubWindow((int)toolbar_.window_coords_.getWidth(), 0, camera_->left_eye()->Width(), camera_->left_eye()->Height(), width, height);
  windows_[1] = SubWindow((int)toolbar_.window_coords_.getWidth() + width, 0, camera_->right_eye()->Width(), camera_->right_eye()->Height(), width, height);


  windows_[2] = SubWindow((int)toolbar_.window_coords_.getWidth(),
                          height, 
                          (int)windows_[2].window_coords_.getWidth(),
                          (int)windows_[2].window_coords_.getHeight());
  

  windows_[3] = SubWindow((int)toolbar_.window_coords_.getWidth() + (int)windows_[2].window_coords_.getWidth(),
                          height, 
                          (int)windows_[3].window_coords_.getWidth(),
                          (int)windows_[3].window_coords_.getHeight());


  //left_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));
  //right_external_framebuffer_.reset(new gl::Fbo(camera_->left_eye()->Width(), camera_->left_eye()->Height()));

  resize();

}

void TTrackApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  toolbar_ = SubWindow(0, 0, 300, 700);
  
  auto ui = ttrk::UIController::Instance();
  if (!ui.IsInitialized()) ui.Initialize("title", toolbar_.framebuffer_->getWidth() - (2 * 20), toolbar_.framebuffer_->getHeight() - 20);

  force_new_frame_ = false;

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

  }
  
  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));
  
}

void TTrackApp::update(){

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  models_to_draw_.clear();
  ttrack.GetUpdate(models_to_draw_, force_new_frame_);

  if (ttrack.IsDone()) {
    shutdown();
    quit();
    return;
  }  
  
  force_new_frame_ = false;

  boost::shared_ptr<const sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<const sv::StereoFrame>(ttrack.GetPtrToCurrentFrame());
 
  windows_[0].texture_ = ci::fromOcv(stereo_frame->GetLeftImage());
  windows_[1].texture_ = ci::fromOcv(stereo_frame->GetRightImage());

}

void TTrackApp::shutdown(){

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

  shader_.bind();
  shader_.uniform("tex0", 0);

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    //glEnable(GL_BLEND);
    //gl::color(1.0f, 1.0f, 1.0f, 0.5f); // Full Brightness, 50% Alpha ( NEW )
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    models_to_draw_[i]->RenderTexture(0);
    //glDisable(GL_BLEND);
    //gl::color(1.0f, 1.0f, 1.0f, 1.0f);
  }

  shader_.unbind();

  cam->ShutDownCameraAfterDrawing();

  ci::gl::popMatrices();

  framebuffer->unbindFramebuffer();
}

void TTrackApp::saveResults(){
  
  auto &ttrack = ttrk::TTrack::Instance(); 
  ttrack.SaveFrame(toOcv(windows_[0].framebuffer_->getTexture()), true);
  ttrack.SaveResults();
 
}

void TTrackApp::drawHelpWindow(boost::shared_ptr<ci::gl::Fbo> framebuffer){

  static ci::gl::Texture help_frame(ci::loadImage(loadResource(HELPER_WIN)));
  //framebuffer->bindFramebuffer();

  //glViewport(0, 0, help_frame.getWidth(), help_frame.getHeight());
  drawBackground(help_frame);
  //gl::draw(help_frame);
  //framebuffer->unbindFramebuffer();

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
  else{
    drawHelpWindow(windows_[0].framebuffer_);
    return;
  }

  if (ttrack.HasConverged()){
    saveResults();
  }

  glViewport(0, 0, width_, height_);

  gl::draw(toolbar_.framebuffer_->getTexture(), ci::Rectf(toolbar_.window_coords_.x1, toolbar_.window_coords_.y2, toolbar_.window_coords_.x2, toolbar_.window_coords_.y1));

  for (auto i = 0; i < windows_.size(); ++i){
  
    gl::Texture tex = windows_[i].framebuffer_->getTexture();
    if (tex.getWidth() != windows_[i].window_coords_.getWidth() || tex.getHeight() != windows_[i].window_coords_.getHeight()){

      cv::Mat x = toOcv(tex);
      cv::Mat x_;
      cv::resize(x, x_, cv::Size(windows_[i].window_coords_.getWidth(), windows_[i].window_coords_.getHeight()));
      tex = fromOcv(x_);

    }
      
    gl::draw(tex, ci::Rectf(windows_[i].window_coords_.x1, windows_[i].window_coords_.y2, windows_[i].window_coords_.x2, windows_[i].window_coords_.y1));
    
  }

}

void TTrackApp::drawToolbar() {

  toolbar_.framebuffer_->bindFramebuffer();

  glViewport(0, 0, width_, height_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  gl::color(1, 1, 1);

  const int width = toolbar_.framebuffer_->getWidth();
  const int height = toolbar_.framebuffer_->getHeight();
  const int buffer = 20;

  //draw line from 20 - 20 to
  
  gl::drawLine(ci::Vec2f((float)buffer, (float)buffer), ci::Vec2f((float)(width - 2 * buffer), (float)buffer));
  gl::drawLine(ci::Vec2f((float)(width - 2 * buffer), (float)buffer), ci::Vec2f((float)(width - 2 * buffer), (float)(height - buffer)));
  gl::drawLine(ci::Vec2f((float)(width - 2 * buffer), (float)(height - buffer)), ci::Vec2f((float)buffer, (float)(height - buffer)));
  gl::drawLine(ci::Vec2f((float)buffer, (float)(height - buffer)), ci::Vec2f((float)buffer, (float)buffer));
  

  auto ui = ttrk::UIController::Instance();
  auto menubar = ui.Menubar();
  menubar.draw();
  
  //menubar_ = ci::params::InterfaceGl::create(ci::app::getWindow(), "Menubar", toPixels(ci::Vec2i(width - 2 * buffer, height - buffer)));

  toolbar_.framebuffer_->unbindFramebuffer();

  
  //for (auto v : i.GetVars())
  //  menubar_->implAddParam(v->GetName(), v->GetValPtr(), v->GetValType(), "min = " + v->GetMin() + " max = " + v->GetMax() + " step = " + v->GetIncrement(), false);
  
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
    for (auto &val : vals){
      if (val > largest_error){
        largest_error = val;
      }
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

  gl::clear(Color(0.1, 0.1, 0.1));

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  ci::CameraPersp maya;
  static bool first = true;
  if (first){
    CameraPersp cam;
    cam.setEyePoint(Vec3f(77.7396, -69.9107, -150.47f));
    cam.setOrientation(ci::Quatf(ci::Vec3f(0.977709, -0.0406959, 0.205982), 2.75995));
    maya_cam_.setCurrentCam(cam);
    first = false;
  }
  
  gl::pushMatrices();
  gl::setMatrices(maya_cam_.getCamera());

  ci::Area viewport = gl::getViewport();
  gl::setViewport(ci::Area(0, 0, framebuffer->getWidth(), framebuffer->getHeight()));

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
  
  drawCamera(camera_view);
  
  cam->SetupLight();

  shader_.bind();
  shader_.uniform("tex0", 0);   

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    models_to_draw_[i]->RenderTexture(0);
  }

  shader_.unbind();

  cam->ShutDownCameraAfterDrawing();

  gl::setViewport(viewport);
  gl::popMatrices();

  framebuffer->unbindFramebuffer();
/*
  cv::Mat a = toOcv(framebuffer->getTexture());*/
  //cv::imwrite("z:/a.png", a);

}

void TTrackApp::drawImageOnCamera(const gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br){

  ci::gl::SaveTextureBindState saveBindState(image_data.getTarget());
  ci::gl::BoolState saveEnabledState(image_data.getTarget());
  ci::gl::ClientBoolState vertexArrayState(GL_VERTEX_ARRAY);
  ci::gl::ClientBoolState texCoordArrayState(GL_TEXTURE_COORD_ARRAY);
  image_data.enableAndBind();

  glEnableClientState(GL_VERTEX_ARRAY);
  GLfloat verts[12];
  glVertexPointer(3, GL_FLOAT, 0, verts);

  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  GLfloat texCoords[8];
  glTexCoordPointer(2, GL_FLOAT, 0, texCoords);

  for (int i = 0; i < 3; ++i) { verts[0 * 3 + i] = tl[i]; }
  for (int i = 0; i < 3; ++i) { verts[1 * 3 + i] = bl[i]; }
  for (int i = 0; i < 3; ++i) { verts[2 * 3 + i] = tr[i]; }
  for (int i = 0; i < 3; ++i) { verts[3 * 3 + i] = br[i]; }

  texCoords[0 * 2 + 0] = 0; texCoords[0 * 2 + 1] = 0;
  texCoords[1 * 2 + 0] = 0; texCoords[1 * 2 + 1] = 1;
  texCoords[2 * 2 + 0] = 1; texCoords[2 * 2 + 1] = 0;
  texCoords[3 * 2 + 0] = 1; texCoords[3 * 2 + 1] = 1;

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void TTrackApp::drawCamera(const gl::Texture &image_data){

  ci::Vec3f vertex[8];
 
  ci::Vec3f eye(0, 0, 0);
  ci::Vec3f bottom_left(-5, -5, 20); 
  ci::Vec3f bottom_right(5, -5, 20); 
  ci::Vec3f top_left(-5, 5, 20);
  ci::Vec3f top_right(5, 5, 20); 

  if (image_data)
    drawImageOnCamera(image_data, top_left, bottom_left, top_right, bottom_right);

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, &vertex[0].x);

  vertex[0] = eye;
  vertex[1] = bottom_left;
  vertex[2] = eye;
  vertex[3] = bottom_right;
  vertex[4] = eye;
  vertex[5] = top_left;
  vertex[6] = eye;
  vertex[7] = top_right;
  glDrawArrays(GL_LINES, 0, 8);

  glLineWidth(2.0f);
  vertex[0] = bottom_left;
  vertex[1] = bottom_right;
  vertex[2] = top_right;
  vertex[3] = top_left;
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glLineWidth(1.0f);
  glDisableClientState(GL_VERTEX_ARRAY);

  size_t MF = 6;
  bottom_left *= MF;
  bottom_right *= MF;
  top_left *= MF;
  top_right *= MF;

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

    shutdown();
    quit();

  } 

  if (k_event.getChar() == 'n'){

    force_new_frame_ = true;

  }
  
}

void TTrackApp::mouseMove(MouseEvent m_event){
  // keep track of the mouse
  mouse_pos_ = m_event.getPos();
}

void TTrackApp::mouseDown(MouseEvent m_event){
  
  // let the camera handle the interaction
  if (windows_[3].window_coords_.contains(m_event.getPos()))
    maya_cam_.mouseDown(m_event.getPos());
}

void TTrackApp::mouseDrag(MouseEvent m_event){

  if (windows_[3].window_coords_.contains(m_event.getPos())){
    // keep track of the mouse
    mouse_pos_ = m_event.getPos();
    // let the camera handle the interaction
    maya_cam_.mouseDrag(m_event.getPos(), m_event.isLeftDown(), m_event.isMiddleDown(), m_event.isRightDown());
  }

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

