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

  const std::string root_dir = "../data/lnd";

  auto &ttrack = ttrk::TTrack::Instance();
  ttrack.SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO, root_dir + "/left.avi",root_dir + "/right.avi");
  
  time_ = getElapsedSeconds();
 
  shader_ = gl::GlslProg( loadResource( RES_SHADER_VERT ), loadResource( RES_SHADER_FRAG ) );

  // load the texture
  //  (note: enable mip-mapping by default)
		
  
  //ImageSourceRef img = loadImage( loadResource( RES_DUCKY_TEX ) );
  //if(img) mTexture = gl::Texture( img, format );
 
  
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
  
  cv::Mat test = cv::Mat::ones(2000,2000,CV_8UC3);
  for(int r=0;r<test.rows;++r){
    for(int c=0;c<test.cols;++c){

      test.at<cv::Vec3b>(r,c) = cv::Vec3b(255,255,255);

    }
  }
  ci::ImageSourceRef img = ci::fromOcv( test );//irs_->first->GetImageROI() );
  frame_texture_ = gl::Texture( img );

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
  
 //draw2D();

	draw3D();

}


void TTrackApp::keyDown( KeyEvent event ){

  static int frame = 0;

  if(event.getChar() == ' '){

    //gl::Fbo framebuffer(640,480,gl::Fbo::Format());//,true,true,true);
    //auto display = this->getDefaultRenderer();
    //auto surface = display->copyWindowSurface(ci::Area(0,0,200,200));
    ////ci::writeImage(surface,  
    

    float rand_1 = -0.5*(float)rand()/(3*RAND_MAX) + (float)rand()/(3*RAND_MAX);
    float rand_2 = -0.5*(float)rand()/(3*RAND_MAX) + (float)rand()/(3*RAND_MAX);
    float rand_3 = -0.5*(float)rand()/(3*RAND_MAX) + (float)rand()/(3*RAND_MAX);

    

    for(auto model = irs_->second.begin(); model != irs_->second.end(); ++model)  {
      auto thing = boost::dynamic_pointer_cast<ttrk::IntuitiveSurgicalLND>(model->PtrToModel());
      thing->RotateHead(rand_1);
      thing->RotateClaspers(rand_2,rand_3);
      //auto meshes_textures_transforms = model->PtrToModel()->GetRenderableMeshes();
      //ci::Matrix44d current_pose = model->CurrentPose().AsCiMatrix();

    }
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
