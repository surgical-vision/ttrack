#include "../../include/utils/renderer.hpp"
#include <cinder/app/App.h>

using namespace ttrk;

Renderer &Renderer::Instance() {
  if (!constructed_){
    instance_.reset(new Renderer());
    constructed_ = true;
  }
  return *(instance_.get());
}

void Renderer::DrawMesh(boost::shared_ptr<Model> mesh, cv::Mat &canvas, cv::Mat &z_buffer, const Pose &pose, const boost::shared_ptr<MonocularCamera> camera){

  Renderer &r = Renderer::Instance();
  while (!r.AddModel(mesh, pose)){ Sleep(5); }

  while (!r.RetrieveRenderedModel(canvas)){ Sleep(5); }

}

bool Renderer::AddModel(boost::shared_ptr<Model> mesh, const Pose &pose){
  
  WriteLock w_lock(mutex);

  if (to_render == nullptr){
    to_render.reset(new Renderable(mesh, pose));
    return true;
  }
  else{
    return false;
  }
  
}


bool Renderer::RetrieveRenderedModel(cv::Mat &canvas,cv::Mat &z_buffer){
  
  WriteLock w_lock(mutex);
  
  if (rendered == nullptr){
    return false;
  }
  else{
    canvas = rendered->canvas_;
    z_buffer = rendered->z_buffer_;
    rendered.release();
    return true;
  }

}

/******** SETUP CODE ***********/

bool Renderer::constructed_ = false;
Lock Renderer::mutex;

boost::scoped_ptr<Renderer> Renderer::instance_(new Renderer);

Renderer::Renderer(){}

void Renderer::Destroy(){

  instance_.reset();
  constructed_ = false;

}