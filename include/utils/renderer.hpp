#ifndef __RENDERER_HPP__
#define __RENDERED_HPP__
#include "../headers.hpp"
#include "../track/model/model.hpp"

namespace ttrk {

  class Renderer {

  public:

    static void DrawMesh(const boost::shared_ptr<Model> mesh, cv::Mat &canvas, const Pose &pose, const boost::shared_ptr<MonocularCamera> camera) {
      /*
      auto mesh_in_camera_coords = mesh->Points(pose);
      for(auto mesh_element = mesh_in_camera_coords->begin(); mesh_element != mesh_in_camera_coords->end(); ++mesh_element) {
        
        Triangle *triangle = dynamic_cast<Triangle *>(*mesh_element);
        cv::Point v1 = camera->ProjectPointToPixel((cv::Point3d)triangle->v1);
        cv::Point v2 = camera->ProjectPointToPixel((cv::Point3d)triangle->v2);
        cv::Point v3 = camera->ProjectPointToPixel((cv::Point3d)triangle->v3);

        if(canvas.channels() == 3)
          cv::line(canvas,v1,v2,cv::Scalar(241,13,156),1,CV_AA);
        else
          cv::line(canvas,v1,v2,cv::Scalar(241),1,CV_AA);
        
      }
      */
    }

  };

}

#endif
