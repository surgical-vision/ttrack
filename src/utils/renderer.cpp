#include <cinder/app/App.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include "../../include/utils/renderer.hpp"

using namespace ttrk;

Renderer &Renderer::Instance() {
  if (!constructed_){
    instance_.reset(new Renderer());
    constructed_ = true;
  }
  return *(instance_.get());
}

void Renderer::DrawMesh(boost::shared_ptr<Model> mesh, cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image, const Pose &pose, const boost::shared_ptr<MonocularCamera> camera){

  Renderer &r = Renderer::Instance();
  while (!r.AddModel(mesh, pose)){ boost::this_thread::sleep(boost::posix_time::milliseconds(100)); }// Sleep(5);

  while (!r.RetrieveRenderedModel(canvas, z_buffer, binary_image)){ boost::this_thread::sleep(boost::posix_time::milliseconds(100)); }

}

void Renderer::DrawStereoMesh(boost::shared_ptr<Model> mesh, cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image, const Pose &pose, const boost::shared_ptr<MonocularCamera> camera){
  
  Renderer &r = Renderer::Instance();
  while (!r.AddModel(mesh, pose)){ boost::this_thread::sleep(boost::posix_time::milliseconds(100)); }// Sleep(5);

  while (!r.RetrieveRenderedModel(canvas, z_buffer, binary_image)){ boost::this_thread::sleep(boost::posix_time::milliseconds(100)); }

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


bool Renderer::RetrieveRenderedModel(cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image){
    
  WriteLock w_lock(mutex);
  
  if (rendered == nullptr){
    return false;
  }
  else{
    canvas = rendered->canvas_;
    z_buffer = rendered->z_buffer_;
    binary_image = rendered->binary_;
    //rendered.release();
    rendered.reset();
    return true;
  }

}

bool Renderer::RetrieveStereoRenderedModel(cv::Mat &left_canvas, cv::Mat &right_canvas, cv::Mat &left_z_buffer, cv::Mat &right_z_buffer, cv::Mat &left_binary_image, cv::Mat &right_binary_image){
  
  WriteLock w_lock(mutex);

  if (rendered == nullptr){
    return false;
  }
  else{
    left_canvas = rendered->canvas_;
    right_canvas = rendered->right_canvas_;
    left_z_buffer = rendered->z_buffer_;
    right_z_buffer = rendered->right_z_buffer_;
    left_binary_image = rendered->binary_;
    right_binary_image = rendered->right_binary_;
    //rendered.release();
    rendered.reset();
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