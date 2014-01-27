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

using namespace ci;
using namespace ci::app;

void TTrackApp::setup(){

  const std::string root_dir = "../data/lnd";

  //t_track_ = ttrk::TTrack::Instance();
  auto &tt = ttrk::TTrack::Instance();
  //t_track_->SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO, root_dir + "/left.avi",root_dir + "/right.avi");
  tt.SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO, root_dir + "/left.avi",root_dir + "/right.avi");
  
  time_ = getElapsedSeconds();
    // load and compile the shader
  //  (note: a shader is not required, but gives a much better visual appearance.
  //	See for yourself by disabling the 'mShader.bind()' call in the draw method.)
  shader_ = gl::GlslProg( loadResource( RES_SHADER_VERT ), loadResource( RES_SHADER_FRAG ) );

  // load the texture
  //  (note: enable mip-mapping by default)
  gl::Texture::Format format;
  format.enableMipmapping(true);			
  //ImageSourceRef img = loadImage( loadResource( RES_DUCKY_TEX ) );
  //if(img) mTexture = gl::Texture( img, format );

  //boost::thread TTrackThread(boost::ref(*t_track_.get()));
  //(*t_track_)();
  tt();
  
  //CameraStereo cam;
  CameraPersp cam;
  cam.setEyePoint( Vec3f(0.0f, 0.0f, 0.0f) );
  cam.setCenterOfInterestPoint( Vec3f(0.0f, 0.0f, 10.0f) );
  cam.setPerspective( 90.0f, getWindowAspectRatio(), 1.0f, 1000.0f );
  maya_cam_.setCurrentCam( cam );

  int width,height;
  //t_track_->GetWindowSize(width,height);
  setWindowSize(width,height);

}

void TTrackApp::update(){

  double elapsed = getElapsedSeconds() - time_;
  time_ = getElapsedSeconds();

  auto &tt = ttrk::TTrack::Instance();
  tt.GetLatestUpdate(irs_);
  //t_track_->GetLatestUpdate(irs_);

}

void TTrackApp::draw(){

  auto area = gl::getViewport();
  

  gl::clear( Colorf(0.5f, 0.5f, 0.5f) );
  // set up the camera 
  gl::pushMatrices();
  gl::setMatrices( maya_cam_.getCamera() );

  // enable the depth buffer (after all, we are doing 3D)
  gl::enableDepthRead();
  gl::enableDepthWrite();

  // draw the grid on the floor
  drawGrid();

  // bind the texture
  //mTexture.bind();

  // bind the shader and tell it to use our texture
  shader_.bind();
  shader_.uniform("tex0", 0);

  // draw the mesh 
  //  (note: reset current color to white so the actual texture colors show up)
  gl::color( Color::white() );
  //  (note: apply transformations to the model)

  for(auto model = irs_.second.begin(); model != irs_.second.end(); ++model)  {
    auto meshs_and_transforms = model->PtrToModel()->GetRenderableMeshes();
    ci::Matrix44d current_pose = model->CurrentPose().AsCiMatrix();
    for(auto mesh_and_transform = meshs_and_transforms.begin();mesh_and_transform != meshs_and_transforms.end();++mesh_and_transform){
      gl::pushModelView();
      gl::multModelView(current_pose * mesh_and_transform->second);
      gl::draw( *mesh_and_transform->first );
      gl::popModelView();
    }
  }

  // unbind the shader and texture
  shader_.unbind();
  //mTexture.unbind();

  // perform 3D picking now, so we can draw the intersection as a sphere
  Vec3f pickedPoint, pickedNormal;
  //if( performPicking( &pickedPoint, &pickedNormal ) ) {
  gl::color( ColorAf(0.0f, 1.0f, 0.0f, 0.5f) );
  // draw an arrow to the picked point along its normal
  gl::drawVector( pickedPoint + pickedNormal, pickedPoint );

  // draw bubbles 
  gl::color( Colorf(0.1f, 0.2f, 0.25f) );
  gl::enableAdditiveBlending();

  gl::disableAlphaBlending();

  gl::popMatrices();

}


void TTrackApp::drawGrid(float size, float step){
	gl::color( Colorf(0.2f, 0.2f, 0.2f) );
  for(float i=-size;i<=size;i+=step) {
    gl::drawLine( Vec3f(i, 0.0f, -size), Vec3f(i, 0.0f, size) );
    gl::drawLine( Vec3f(-size, 0.0f, i), Vec3f(size, 0.0f, i) );
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

/*
#include <boost/program_options.hpp> 
#include "../include/utils/helpers.hpp"

#if defined _WIN32 || _WIN64
  #include<direct.h>
#elif defined __linux__
#endif


int main(int argc, char **argv){

  using namespace boost::program_options;

  bool stereo;
  std::string video_file,left_video_file,right_video_file,image_dir,classifier_type_,classifier_file,camera_calibration_file,model_file,results_dir;
  options_description desc("Options"); 
  desc.add_options() 
    ("help", "Print help messages")
    ("classifier_type",value<std::string>(&classifier_type_),"Type of classifier to load. Supported options: RF, SVM.")
    ("classifier_file", value<std::string>(&classifier_file), "Path to the classifier to load.")
    ("stereo", value<bool>( &stereo )->zero_tokens()  , "Flag to switch stereo on.")
    ("video_file",value<std::string>(&video_file),"path to monocular or stereo video file to do pose estimation/tracking on.")
    ("left_video_file",value<std::string>(&left_video_file),"path to left stereo video file to do pose estimation/tracking on.")
    ("right_video_file",value<std::string>(&right_video_file),"path to right stereo video file to do pose estimation/tracking on.")
    ("image_dir",value<std::string>(&image_dir),"path to directory containing images to do pose estimation/tracking on.")
    ("camera_calibration_file",value<std::string>(&camera_calibration_file),"path to directory containing images to do pose estimation/tracking on.")
    ("model_file",value<std::string>(&model_file),"path to file where model calibration is stored.")
    ("results_dir",value<std::string>(&results_dir),"path to directory where images can be saved.")   
    ; 

  variables_map vm; 
  ttrk::CameraType camera_type;
  ttrk::ClassifierType classifier_type;

  try{ 

    store(parse_command_line(argc, argv, desc), vm); // can throw 

    if(vm.count("help")){ 
      std::cout << "Basic Command Line Parameter App" << std::endl << desc << std::endl; 
      ttrk::SAFE_EXIT(0);
    } 

    notify(vm); // throws on error, so do after help in case 

    if(stereo) camera_type = ttrk::STEREO;
    else camera_type = ttrk::MONOCULAR;

    if(classifier_type_ == "RF" || classifier_type_ == "rf")
      classifier_type = ttrk::RF;
    else if (classifier_type_ == "SVM" || classifier_type_ == "svm")
      classifier_type = ttrk::SVM;
    else
      throw(boost::program_options::error("Incorrect classifier type given. Options are: rf, svm."));

    // there are any problems 
  }catch(error& e){ 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cerr << desc << std::endl; 
    ttrk::SAFE_EXIT(1); 
  } 

  
  ttrk::TTrack &t = ttrk::TTrack::Instance();

  try{
    
#if defined(_WIN32) || defined(_WIN64)
     //_chdir("../");
#endif



    //t.SetUp(model_file,camera_calibration_file,classifier_file,results_dir, classifier_type, camera_type,left_video_file);


    if(video_file.length())
      t.SetUp(model_file,camera_calibration_file,classifier_file,results_dir, classifier_type, camera_type,video_file);

    else if (left_video_file.length() && right_video_file.length())
      t.SetUp(model_file,camera_calibration_file,classifier_file,results_dir, classifier_type, camera_type,left_video_file,right_video_file);

    else if (image_dir.length())
      t.SetUp(model_file,camera_calibration_file,classifier_file,results_dir, classifier_type, camera_type,image_dir);

    else
      throw(std::runtime_error("Error, cmd line args\n"));

    t();

  }catch(std::runtime_error &e){

    std::cerr << e.what() << "\n";
    ttrk::SAFE_EXIT(1);
  
  }catch(std::exception &e){
  
    std::cerr << e.what() << "\n";
    ttrk::SAFE_EXIT(1);
  
  }
  
  
  ttrk::TTrack::Destroy();

#if defined(_WIN32) || defined(_WIN64)
  _CrtDumpMemoryLeaks();
#endif
  
  ttrk::SAFE_EXIT(0);

}
*/
