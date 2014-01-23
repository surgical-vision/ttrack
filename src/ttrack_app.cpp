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

using namespace ci;
using namespace ci::app;

void TTrackApp::setup(){

  const std::string root_dir = "../data/lnd";

  t_track_ = ttrk::TTrack::Instance();
  t_track_->SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO);
  
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

  //CameraStereo cam;
  CameraPersp cam;
  cam.setEyePoint( Vec3f(0.0f, 0.0f, 0.0f) );
  cam.setCenterOfInterestPoint( Vec3f(0.0f, 0.0f, 10.0f) );
  cam.setPerspective( 90.0f, getWindowAspectRatio(), 1.0f, 1000.0f );
  maya_cam_.setCurrentCam( cam );


}

void TTrackApp::update(){

  double elapsed = getElapsedSeconds() - time_;
  time_ = getElapsedSeconds();

}

void TTrackApp::draw(){

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

  //for(auto mesh = aj->models_.begin(); mesh != aj->models_.end(); ++mesh)  {
  gl::pushModelView();
  //if( mesh->movable_ ){
  //  gl::multModelView( mesh->getTransform() );
  //}
  //gl::draw( mesh->model_ );
  gl::popModelView();

  //}
  // unbind the shader and texture
  shader_.unbind();
  //mTexture.unbind();

  // perform 3D picking now, so we can draw the intersection as a sphere
  Vec3f pickedPoint, pickedNormal;
  //if( performPicking( &pickedPoint, &pickedNormal ) ) {
  gl::color( ColorAf(0.0f, 1.0f, 0.0f, 0.5f) );
  // draw an arrow to the picked point along its normal
  gl::drawVector( pickedPoint + pickedNormal, pickedPoint );

  // add one bubble at the point of intersection
  //mBubbles.push_back( BubbleRef( new Bubble( pickedPoint ) ) );
  //}

  // draw bubbles 
  gl::color( Colorf(0.1f, 0.2f, 0.25f) );
  gl::enableAdditiveBlending();

  //vector<BubbleRef>::iterator itr;
  //for(itr = mBubbles.begin();itr!=mBubbles.end();++itr)
  //(*itr)->draw();

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
