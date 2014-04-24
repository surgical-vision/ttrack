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

void TTrackApp::setup(){

  const std::string root_dir = "../data/new_surgeon";

  auto &ttrack = ttrk::TTrack::Instance();
  //ttrack.SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO, root_dir + "/left.avi",root_dir + "/right.avi");
  ttrack.SetUp(root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF, ttrk::MONOCULAR, root_dir + "/surgery.m4v");
  time_ = getElapsedSeconds();
 
  shader_ = gl::GlslProg( loadResource( RES_SHADER_VERT ), loadResource( RES_SHADER_FRAG ) );
  
  boost::thread main_thread( boost::ref(ttrack) );
  
  //CameraStereo cam;
  CameraPersp cam;
  cam.setEyePoint( Vec3f(0.0f,0.0f,0.0f) );
  cam.setCenterOfInterestPoint( Vec3f(0.0f, 0.0f, 40.0f) );
  cam.setPerspective( 70.0f, getWindowAspectRatio(), 1.0f, 1000.0f );
  maya_cam_.setCurrentCam( cam );
    
}

void TTrackApp::update(){

  double elapsed = getElapsedSeconds() - time_;
  time_ = getElapsedSeconds();

  auto &ttrack = ttrk::TTrack::Instance();
  if(!ttrack.GetLatestUpdate(irs_))
    return;
  
}

void TTrackApp::draw3D() {

  if(!irs_) return;

  gl::pushMatrices();
  gl::setMatrices( maya_cam_.getCamera() );
  gl::enableAlphaBlending();

  // enable the depth buffer (after all, we are doing 3D)
  gl::enableDepthRead();
  gl::enableDepthWrite();

  // bind the shader and tell it to use our texture
  drawMeshes();
  
  
  //gl::enableAdditiveBlending();

  gl::disableAlphaBlending();

  gl::popMatrices();

}

void TTrackApp::draw2D() {
      
  if( frame_texture_ )
    gl::draw(frame_texture_, getWindowBounds() );

}

void TTrackApp::drawMeshes() {

  gl::color( Color::white() );

  for(auto model = irs_->second.begin(); model != irs_->second.end(); ++model)  {
    auto meshes_textures_transforms = model->PtrToModel()->GetRenderableMeshes();
    ci::Matrix44d current_pose = model->CurrentPose().AsCiMatrix();

    for(auto mesh_tex_trans = meshes_textures_transforms.begin();mesh_tex_trans != meshes_textures_transforms.end();++mesh_tex_trans){
      
      auto texture = mesh_tex_trans->get<1>();
      //texture->bind();
      shader_.bind();
      //shader_.uniform("tex0", 0);
      
      gl::pushModelView();
      gl::multModelView(current_pose * mesh_tex_trans->get<2>());
      
      const auto trimesh = (*mesh_tex_trans).get<0>();
      gl::draw( *trimesh );
      
      gl::popModelView();
      shader_.unbind();
      //texture->unbind();

    }
  }
}


void TTrackApp::draw(){

  gl::clear( Color( 0.0f, 0.0f, 0.0f ) , true ); //set the screen to black and clear the depth buffer
  
  draw2D();

	//draw3D();

}


void TTrackApp::keyDown( KeyEvent event ){

  if(event.getChar() == ' '){

    throw(std::runtime_error("Err"));

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
