#ifndef __RENDERER_HPP__
#define __RENDERED_HPP__
#include "../headers.hpp"
#include "../track/model/model.hpp"
#include <cinder/gl/Fbo.h>
#include "../ttrack.hpp"
#include <boost/thread.hpp>

namespace ttrk {

  typedef boost::shared_mutex Lock;
  typedef boost::unique_lock< Lock > WriteLock;

  struct Renderable {
    
    Renderable(boost::shared_ptr<Model> mesh, const Pose &pose) : mesh_(mesh), pose_(pose) {}

    boost::shared_ptr<Model> mesh_;
    cv::Mat canvas_;
    cv::Mat right_canvas_;
    cv::Mat z_buffer_;
    cv::Mat right_z_buffer_;
    cv::Mat binary_;
    cv::Mat right_binary_;
    Pose pose_;

  };


  class Renderer {

  public:

    static Renderer &Instance();
    static void Destroy();

    static void DrawMesh(boost::shared_ptr<Model> mesh, cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image, const ttrk::Pose &pose, const boost::shared_ptr<MonocularCamera> camera);

    std::unique_ptr<Renderable> to_render;
    std::unique_ptr<Renderable> rendered;
    static Lock mutex;

  protected:

    bool AddModel(boost::shared_ptr<Model> mesh, const Pose &pose);
    bool RetrieveRenderedModel(cv::Mat &canvas, cv::Mat &z_buffer, cv::Mat &binary_image);
    bool RetrieveStereoRenderedModel(cv::Mat &left_canvas, cv::Mat &right_canvas, cv::Mat &left_z_buffer, cv::Mat &right_z_buffer, cv::Mat &left_binary_image, cv::Mat &right_binary_image);
  
  private:

    Renderer();
    
    Renderer(const Renderer &);
    Renderer &operator=(const Renderer &);
    static bool constructed_;
    static boost::scoped_ptr<Renderer> instance_;

  };

}

#endif
