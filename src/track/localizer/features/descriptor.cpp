#include "../../../../include/ttrack/track/localizer/features/descriptor.hpp"

using namespace ttrk;

Descriptor::Descriptor(const cv::Vec3f pt, const cv::Vec3f normal, const cv::Mat &descriptor){

  coordinate_ = ci::Vec3f(pt[0], pt[1], pt[2]);
  normal_ = ci::Vec3f(normal[0], normal[1], normal[2]);
  descriptor_ = descriptor.clone();
  times_matched_ = 0;
  total_num_attempts_ = 0;

}
ci::Vec3f Descriptor::ciGetPointInEyeSpace(Pose &pose) const {

  ci::Vec4f c(coordinate_[0], coordinate_[1], coordinate_[2], 1);
  ci::Matrix44f m = pose;
  ci::Vec4f r = m * c;
  r.normalize();
  return r.xyz();

}

//void Descriptor::ConstructFromModelDescriptor(const boost::shared_ptr<Model> model, const LearnedDescriptor &model_descriptor){
//
//  coordinate_ = model->GetBasePose().TransformPoint(model_descriptor.ciGetModelSpaceCoordinate());
//  normal_ = model->GetBasePose().TransformPoint(model_descriptor.ciGetModelSpaceNormal());
//  descriptor_ = model_descriptor.GetDescriptor();
//  times_matched_ = model_descriptor.GetTimesMatched();
//  total_num_attempts_ = model_descriptor.GetTotalNumberOfAttempts();
//
//}

bool Descriptor::ShouldUseThisDescriptor(Pose &pose) const {

  //check visibility and also check normals

  return true;

}