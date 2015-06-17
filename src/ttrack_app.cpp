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
#include <cinder/Text.h>

#ifdef USE_MATHGL
#include <mgl2/mgl.h>
#endif

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

  const std::string results_dir = reader.get_element("output-dir");
  std::string output_dir_this_run = results_dir;
  int n = 0;
  while (boost::filesystem::is_directory(output_dir_this_run)){
    std::stringstream res;
    res << results_dir << "/run_" << n;
    output_dir_this_run = res.str();
    ++n;
  }

  //throwing errors here? did you remember the zero at element 15 of start pose or alteranatively set the trackable dir to absolute in the cfg file
  ttrack.SetUp(reader.get_element("trackable"),
               root_dir + "/" + reader.get_element("camera-config"),
               root_dir + "/" + reader.get_element("classifier-config"),
               output_dir_this_run,
               ttrk::TTrack::LocalizerTypeFromString(reader.get_element("localizer-type")),
               ttrk::TTrack::ClassifierFromString(reader.get_element("classifier-type")),
               root_dir + "/" + reader.get_element("left-input-video"),
               root_dir + "/" + reader.get_element("right-input-video"),
               starting_poses,
               number_of_labels);
  
  camera_.reset(new ttrk::StereoCamera(root_dir + "/" + reader.get_element("camera-config")));
  
  windows_[0].Init("Left Eye", toolbar_.GetRect().x2, 0, camera_->left_eye()->Width(), camera_->left_eye()->Height(), 625, 500, false);
  windows_[1].Init("Right Eye", toolbar_.GetRect().x2, windows_[0].GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), 625, 500, false);

  windows_[2].Init("Left Classification", 0, toolbar_.GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), toolbar_.GetRect().getWidth(), toolbar_.GetRect().getWidth(), false);
  windows_[3].Init("Right Classification", 0, toolbar_.GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), toolbar_.GetRect().getWidth(), toolbar_.GetRect().getWidth(), false);
  windows_[4].Init("Localizer Output", 0, toolbar_.GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), toolbar_.GetRect().getWidth(), toolbar_.GetRect().getWidth(), false);
  windows_[5].Init("3D Viewer", 0, toolbar_.GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), toolbar_.GetRect().getWidth(), toolbar_.GetRect().getWidth(), false);
  windows_[6].Init("Plotter", 0, toolbar_.GetRect().y2, camera_->left_eye()->Width(), camera_->left_eye()->Height(), toolbar_.GetRect().getWidth(), toolbar_.GetRect().getWidth(), false);

  ttrk::SubWindow::output_directory = output_dir_this_run;

  show_extra_view_ = EXTRA_VIEW_3D;

  resize();

}

void TTrackApp::startTracking(){

  run_tracking_ = !run_tracking_;

}

void TTrackApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  const size_t width_of_toolbar = 375;

  toolbar_.Init("GUI", 0, 0, width_of_toolbar, 500, false);
  

  //create left eye window
  for (size_t i = 0; i < 7; ++i)
    windows_.push_back(ttrk::SubWindow());

  windows_[0].Init("Left Eye", toolbar_.GetRect().x2, 0, 625, 500, true);
  windows_[1].Init("Right Eye", toolbar_.GetRect().x2, windows_[0].GetRect().y2, 625, 500, true);
  windows_[2].Init("Left Classification", 0, toolbar_.GetRect().y2, 625, 500, width_of_toolbar, width_of_toolbar, true);
  windows_[3].Init("Right Classification", 0, toolbar_.GetRect().y2, 625, 500, width_of_toolbar, width_of_toolbar, true);
  windows_[4].Init("Localizer Output", 0, toolbar_.GetRect().y2, 625, 500, width_of_toolbar, width_of_toolbar, true);
  windows_[5].Init("3D Viewer", 0, toolbar_.GetRect().y2, 625, 500, width_of_toolbar, width_of_toolbar, true);
  windows_[6].Init("Plotter", 0, toolbar_.GetRect().y2, 625, 500, width_of_toolbar, width_of_toolbar, true);


  auto &ui = ttrk::UIController::Instance();
  if (!ui.IsInitialized()) {
    ui.Initialize("ToolBar", toolbar_.GetRect().getWidth(), toolbar_.GetRect().getHeight());

    ui.AddFunction("Start Tracking", std::bind(&TTrackApp::startTracking, this));

    ui.AddFunction("Quit application", std::bind(&TTrackApp::shutdown, this));
    ui.AddFunction("View 3D Scene", std::bind(&TTrackApp::show3DScene, this));
    ui.AddFunction("View Error Function", std::bind(&TTrackApp::showPlotter, this));
    ui.AddFunction("View Left Eye Detector Output", std::bind(&TTrackApp::showDetectorOutput, this));
    ui.AddFunction("View Localizer Output", std::bind(&TTrackApp::showLocalizerOutput, this));
    ui.AddFunction("Reset 3D Scene", std::bind(&TTrackApp::reset3DViewerPosition, this));

    ui.AddSeparator();

    auto bind_both = [this]() { windows_[2].InitSavingWindow(); windows_[3].InitSavingWindow(); };

    ui.AddFunction("Save Left Eye", std::bind(&ttrk::SubWindow::InitSavingWindow, &(windows_[0])));
    ui.AddFunction("Save Right Eye", std::bind(&ttrk::SubWindow::InitSavingWindow, &(windows_[1])));
    ui.AddFunction("Save Classification", std::bind(bind_both));
    ui.AddFunction("Save Localizer Output", std::bind(&ttrk::SubWindow::InitSavingWindow, &(windows_[4])));
    ui.AddFunction("Save 3D Viewer Output", std::bind(&ttrk::SubWindow::InitSavingWindow, &(windows_[5])));

    ui.AddSeparator();

  }

  force_new_frame_ = false;
  reset_3D_viewport_ = true;
  run_tracking_ = false;
   
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
  if (!ttrack.IsRunning() || !run_tracking_) return;

  models_to_draw_.clear();
  ttrack.GetUpdate(models_to_draw_, force_new_frame_);

  if (ttrack.IsDone()) {
    shutdown();
    quit();
    return;
  }  
  
  force_new_frame_ = false;

  boost::shared_ptr<const sv::StereoFrame> stereo_frame = boost::dynamic_pointer_cast<const sv::StereoFrame>(ttrack.GetPtrToCurrentFrame());
  left_eye_image_ = ci::fromOcv(stereo_frame->GetLeftImage());
  right_eye_image_ = ci::fromOcv(stereo_frame->GetRightImage());

  cv::Mat d = ttrack.GetCurrentDetectorImage();
    
  if (!d.empty()){
   
    cv::Mat left_d = d(cv::Rect(0, 0, d.cols / 2, d.rows)).clone();
    cv::Mat right_d = d(cv::Rect(d.cols / 2, 0, d.cols / 2, d.rows)).clone();

    left_detector_image_ = ci::fromOcv(left_d);
    right_detector_image_ = ci::fromOcv(right_d);

  }

  cv::Mat l = ttrack.GetCurrentLocalizerImage();
  if (!l.empty())
    localizer_image_ = ci::fromOcv(l);

}

void TTrackApp::shutdown(){

  ttrk::TTrack::Destroy();

  cinder::app::AppNative::shutdown();

  cinder::app::AppNative::quit();

}

void TTrackApp::drawBackground(gl::Texture &background, const cv::Size target_size){

  if (!background || background.getWidth() == 0 || background.getHeight() == 0) return;

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glViewport(0, 0, target_size.width, target_size.height);
  glScissor(0, 0, target_size.width, target_size.height);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, target_size.width, 0, target_size.height, 0, 1);

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

void TTrackApp::drawEye(ttrk::SubWindow &window, ci::gl::Texture &background, const boost::shared_ptr<ttrk::MonocularCamera> cam, bool draw_models){

  window.BindAndClear();

  ci::gl::disableDepthRead();

  drawBackground(background, cv::Size(cam->Width(), cam->Height()));

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
  ci::gl::pushMatrices();
    
  cam->SetupCameraForDrawing();

  shader_.bind();
  shader_.uniform("tex0", 0);

  for (size_t i = 0; i < models_to_draw_.size() && draw_models; ++i){

    models_to_draw_[i]->RenderTexture(0);

  }

  shader_.unbind();

  cam->ShutDownCameraAfterDrawing();

  ci::gl::popMatrices();

  window.UnBind();

  window.Draw();

}

void TTrackApp::saveResults(){
  
  auto &ttrack = ttrk::TTrack::Instance(); 
  ttrack.SaveFrame(toOcv(windows_[0].GetContents()), true);
  ttrack.SaveResults();

  for (auto &sub_win : windows_){
    if (sub_win.CanSave()){
      sub_win.WriteFrameToFile();
    }
  }
 
}

void TTrackApp::drawImageContents(ttrk::SubWindow &window, gl::Texture &image){

  if (!image || image.getWidth() == 0 || image.getHeight() == 0) return;

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glViewport(0, 0, window.BufferWidth(), window.BufferHeight());
  glScissor(0, 0, window.BufferWidth(), window.BufferHeight());

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, image.getWidth(), 0, image.getHeight(), 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);

  image.setFlipped(true);

  gl::draw(image);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glViewport(vp[0], vp[1], vp[2], vp[3]);

}

void TTrackApp::drawHelpWindow(ttrk::SubWindow &window){

  static ci::gl::Texture help_frame(ci::loadImage(loadResource(HELPER_WIN)));
  
  drawBackground(window, help_frame);
  
  window.Draw();
   

}

void TTrackApp::drawBackground(ttrk::SubWindow &window, ci::gl::Texture &background){

  window.BindAndClear();
  drawImageContents(window, background);
  window.UnBind();

}

void TTrackApp::drawExtra(){

  drawBackground(windows_[2], left_detector_image_);
  drawBackground(windows_[3], right_detector_image_);
  drawBackground(windows_[4], localizer_image_);
  draw3D(windows_[5], left_eye_image_, camera_->left_eye());
  drawPlotter(windows_[6]);

  if (show_extra_view_ == EXTRA_VIEW_PLOTTER)
    windows_[6].Draw();

  else if (show_extra_view_ == EXTRA_VIEW_3D)
    windows_[5].Draw();

  else if (show_extra_view_ == EXTRA_VIEW_DETECTOR && left_detector_image_)
    windows_[2].Draw();

  else if (show_extra_view_ == EXTRA_VIEW_LOCALIZER && localizer_image_)
    windows_[4].Draw();  

}

void TTrackApp::draw(){

  gl::clear(ci::Color(0, 0, 0), true);

  drawToolbar();

  auto &ttrack = ttrk::TTrack::Instance();

  if (ttrack.IsRunning() && run_tracking_){

    drawEye(windows_[0], left_eye_image_, camera_->left_eye());
    drawEye(windows_[1], right_eye_image_, camera_->right_eye());
    
    drawExtra();
    
    if (ttrack.HasConverged()){
      saveResults();
    }

  }
  else{
    drawHelpWindow(windows_[0]);
    return;
  }

}

void TTrackApp::drawToolbar() {

  auto &ui = ttrk::UIController::Instance();
  toolbar_.Draw(ui.Menubar());
  
}

void TTrackApp::drawPlotter(ttrk::SubWindow &window) {

  const int width = window.BufferWidth();
  const int height = window.BufferHeight();

  gl::Texture tex(720, 576);

#ifndef USE_MATHGL

  mglGraph graph(0, width, height);

  graph.Title("Localizer error");
  graph.Alpha(true);
  graph.Light(true);

  size_t number_of_frames = 0;
  float max_element = std::numeric_limits<float>::min();
  float min_element = std::numeric_limits<float>::max();

  graph.Label('x', "Frame No", 0);
  graph.Label('y', "Error", 0);

  auto &plotter = ttrk::ErrorMetricPlotter::Instance();
  auto &plots = plotter.GetPlottables();

  for (auto &plot : plots){
    std::vector<float> &vals = plot->GetErrorValues();
    if (vals.size() > number_of_frames) number_of_frames = vals.size();
    float max_element_of_plot = *std::max_element(vals.begin(), vals.end());
    float min_element_of_plot = *std::min_element(vals.begin(), vals.end());

    if (max_element_of_plot > max_element)
      max_element = max_element_of_plot;
    if (min_element_of_plot < min_element)
      min_element = min_element_of_plot;

  }


  //graph.SetRange('x', 0, number_of_frames);
  //graph.SetRange('y', min_element, max_element);
  graph.SetRanges(0, number_of_frames, min_element, max_element);
  graph.Axis();

  
  for (auto &plot : plots){

    std::vector<float> &vals = plot->GetErrorValues();

    //no stl in debug mode for mgl
    float *data = new float[vals.size()];
    for (int i = 0; i < vals.size(); ++i)
      data[i] = vals[i];

    mglData error_data(data, vals.size());
    graph.Plot(error_data);

    delete[] data;

  }

  unsigned char *buf = new uchar[4 * width*height];
  graph.GetBGRN(buf, 4 * width*height);

  cv::Mat m(cv::Size(width, height), CV_8UC4, (void *)buf);

  std::vector<cv::Mat> chans;
  cv::split(m, chans);
  chans.pop_back();
  cv::merge(chans, m);

  tex = fromOcv(m);

  window.BindAndClear();
  
  drawImageContents(window, tex);
  
  window.UnBind();

  delete[] buf;

#else

  static ci::gl::Texture help_frame(ci::loadImage(loadResource(ENABLED_WIN)));

  drawBackground(window, help_frame);

  window.Draw();
  
#endif



} 

void TTrackApp::draw3D(ttrk::SubWindow &window, const gl::Texture &camera_view, const boost::shared_ptr<ttrk::MonocularCamera> cam) {

  window.BindAndClear();

  const int width = window.BufferWidth();
  const int height = window.BufferHeight();

  auto &ttrack = ttrk::TTrack::Instance();
  if (!ttrack.IsRunning()) return;

  ci::Vec3f test_start_pose(10, -5, 60);

  ci::CameraPersp maya;
  
  if (reset_3D_viewport_){
    CameraPersp cam;
    cam.setEyePoint(Vec3f(77.7396, -69.9107, -150.47f));
    cam.setOrientation(ci::Quatf(ci::Vec3f(0.977709, -0.0406959, 0.205982), 2.75995));
    maya_cam_.setCurrentCam(cam);
    reset_3D_viewport_ = false;
  }
  
  gl::pushMatrices();
  gl::setMatrices(maya_cam_.getCamera());

  ci::Area viewport = gl::getViewport();
  gl::setViewport(ci::Area(0, 0, width, height));

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
  
  drawCamera(camera_view);
  
  cam->SetupLight();

  shader_.bind();
  shader_.uniform("tex0", 0);   

  for (size_t i = 0; i < models_to_draw_.size(); ++i){
    //gl::pushModelView();
    //gl::multModelView(maya_cam_.getCamera().getModelViewMatrix().inverted());
    models_to_draw_[i]->RenderTexture(0);
    //gl::popModelView();
  }

  shader_.unbind();

  cam->ShutDownCameraAfterDrawing();

  gl::popModelView();

  //gl::popMatrices();

  gl::setViewport(viewport);
  gl::popMatrices();

  window.UnBind();

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
  if (windows_[2].GetRect().contains(m_event.getPos()) && show_extra_view_ == EXTRA_VIEW_3D )
    maya_cam_.mouseDown(m_event.getPos());

}

void TTrackApp::mouseDrag(MouseEvent m_event){

  if (windows_[2].GetRect().contains(m_event.getPos()) && show_extra_view_ == EXTRA_VIEW_3D){
    // keep track of the mouse
    mouse_pos_ = m_event.getPos();
    // let the camera handle the interaction
    maya_cam_.mouseDrag(m_event.getPos(), m_event.isLeftDown(), m_event.isMiddleDown(), m_event.isRightDown());
  }

}

void TTrackApp::resize(){

  const int width = toolbar_.GetRect().getWidth() + windows_[0].GetRect().getWidth();

  int height = toolbar_.GetRect().getHeight() + windows_[2].GetRect().getHeight();
  height = std::max(height, (int)(windows_[0].GetRect().getHeight() + windows_[1].GetRect().getHeight()));


  setWindowSize(width, height);

  width_ = width;
  height_ = height;

  CameraPersp cam = maya_cam_.getCamera();
  cam.setAspectRatio( getWindowAspectRatio() );
  maya_cam_.setCurrentCam( cam );
}

CINDER_APP_NATIVE( TTrackApp, RendererGl )

