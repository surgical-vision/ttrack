#include "../../../include/track/articulated/articulated_level_set.hpp"

using namespace ttrk;


void ArticulatedLevelSet::TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame){

  std::vector<Node::Ptr> nodes;

  size_t idx = 1;
  Node::Ptr p = model->GetModel();

  nodes.push_back(p);

  while (1){

    size_t start_idx = 0;
    Node::Ptr p1 = p->GetChildByIdx(start_idx, idx);
    if (p1 == nullptr) break;
    idx += 1;

    nodes.push_back(p1);

  }

  std::vector<ComponentData> thangs;

  for (auto node : nodes){

    for (auto nodeA : nodes){
      nodeA->SetDraw(false);
    }

    node->SetDraw(true);

    thangs.push_back(ComponentData());

    thangs.back().model_ = node;

    if (!node->HasMesh()) continue;

    StereoPWP3D tracker(stereo_camera_);// ->left_eye()->Width(), stereo_camera_->left_eye()->Height());
    tracker.frame_ = frame;

    cv::Mat sdf_image, front_intersection_image, back_intersection_image;
    tracker.ProcessSDFAndIntersectionImage(model, stereo_camera_->left_eye(), sdf_image, front_intersection_image, back_intersection_image);

    thangs.back().sdf_image_ = sdf_image;
    thangs.back().front_intersection_image_ = front_intersection_image;
    thangs.back().back_intersection_image_ = back_intersection_image;

  }

  cv::Mat composite_sdf(cv::Size(frame->cols(),frame->rows()), CV_32FC2);

  for (int r = 0; r < composite_sdf.rows; ++r){

    for (int c = 0; c < composite_sdf.cols; ++c){

      float min_val = std::numeric_limits<float>::max();

      for (size_t t = 0; t < thangs.size(); ++t){

        if (thangs[t].model_->HasMesh() == 0) continue;

        float val = std::abs(thangs[t].sdf_image_.at<float>(r, c));
        if (val <= min_val){
          min_val = thangs[t].sdf_image_.at<float>(r, c);
          composite_sdf.at<cv::Vec2f>(r, c) = cv::Vec2f(min_val, t);
        }
      
      }

    }

  }

  int AXS = 0;

  //load multiclass detection module where each pixels is labelled (0:background, 1: shaft, 2: head, 3: clasper)

  //models is currently a tree of components

  //create a SDF and intersection for each component

  //possible to include some interior surface information by including the signed distnace function around the edge between the instrument shaft and instrument head

  //do normal pwp3d cost function iterating over the pixels

  //for each pixel find the sdf that it corrsponds to 

  //each component can have a colour model (one against all classification)

  //region agreement is computed as same

  //3d intersection values need to be in camera frame and LOCAL coordinate frame of the geometry

  //update the correct part of the jacobian with the derivative

  //shaft -> rigid se3 
  //head -> wrist pitch
  //jointly solve claspers and wrist yaw

}

void ArticulatedLevelSet::ProcessArticulatedSDFAndIntersectionImage(const boost::shared_ptr<Model> mesh, const boost::shared_ptr<MonocularCamera> camera, cv::Mat &sdf_image, cv::Mat &front_intersection_image, cv::Mat &back_intersection_image){

  //generate each signed distance function for each component - this allows us to know which component of the articulated model
  //is 'generating' each value is the composite SDF

  //for (size_t i = 0; i < component_trackers_.size(); ++i){

  //  Model &m = mesh->

  //}




}

