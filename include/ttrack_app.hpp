#ifndef __TTRACK_APP_HPP__
#define __TTRACK_APP_HPP__
#include <cinder/app/AppBasic.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/ImageIo.h>
#include <cinder/MayaCamUI.h>
#include <cinder/Rand.h>
#include <cinder/TriMesh.h>
#include <cinder/ObjLoader.h>
#include <cinder/Json.h>

using namespace ci;
using namespace ci::app;

class TTrackApp : public AppBasic {

public:

  void setup();
  void update();
  void draw();

  //~Picking3DApp() { delete aj; } 

  void drawGrid(float size=100.0f, float step=10.0f);

  //bool performPicking( Vec3f *pickedPoint, Vec3f *pickedNormal );

  void mouseMove( MouseEvent event );
  void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void resize();

protected:
  // shader and texture for our model
  gl::GlslProg	shader_;
  gl::Texture		texture_;

  // our camera
  MayaCamUI	maya_cam_;

  // keep track of the mouse
  Vec2i	mouse_pos_;

  // keep track of time
  double time_;

};

#endif
