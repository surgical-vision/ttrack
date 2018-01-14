#include <cinder/app/App.h>

#include "../../../include/ttrack/track/model/pose.hpp"

using namespace ttrk;

Pose::operator ci::Matrix44f() const {

  //cv::Vec3f euler = rotation_.EulerAngles();
  //ci::Matrix44f ret = ci::Matrix44f::createRotation(ci::Vec3d(euler[0],euler[1],euler[2]));
  ci::Matrix44f ret = rotation_;
    
  ////using http://answers.unity3d.com/storage/temp/12048-lefthandedtorighthanded.pdf
  //  
  ////transform rotation by flipping z
  //ci::Matrix44f S_z;
  //S_z.setToIdentity();
  //S_z.at(2, 2) *= -1; 
  //ret = S_z * ret * S_z;
  //  
  ////transform translation
  //const ci::Vec3f translation_transformed = S_z.subMatrix33(0,0) * ci::Vec3f(translation_[0], translation_[1], translation_[2]);
  
  for (int i = 0; i < 3; i++){
    ret.at(i, 3) = translation_[i];
  }
    
  return ret;

}

cv::Vec3f Pose::TransformPoint(const cv::Vec3f &point_in_world_coordinates) const{

  const ci::Vec3f vec(point_in_world_coordinates[0], point_in_world_coordinates[1], point_in_world_coordinates[2]);
  const ci::Vec3f ret = TransformPoint(vec);

  return cv::Vec3f(ret[0], ret[1], ret[2]);

}

cv::Vec3f Pose::InverseTransformPoint(const cv::Vec3f &point_in_model_coordinates) const {

  ci::Matrix44f pose_inv = *this;
  pose_inv.invert();

  const ci::Vec3f vec(point_in_model_coordinates[0], point_in_model_coordinates[1], point_in_model_coordinates[2]);
  const ci::Vec3f ret = pose_inv * vec;

  return cv::Vec3f(ret[0], ret[1], ret[2]);

}

ci::Vec3f Pose::TransformPoint(const ci::Vec3f &point_in_world_coordinates) const{

  ci::Matrix44f pose = *this;

  return pose * point_in_world_coordinates;

}

void Pose::SetPose(std::vector<float> &pose) {

  assert(pose.size() == 7);

  translation_[0] = pose[0];
  translation_[1] = pose[1];
  translation_[2] = pose[2];

  //rotation_ = sv::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  //rotation_ = rotation_.Normalize();
  rotation_ = ci::Quatf(pose[3], pose[4], pose[5], pose[6]);
  rotation_.normalize();

}



std::vector<float> Pose::GetPose() const{

  std::vector<float> ret;
  ret.push_back(translation_[0]);
  ret.push_back(translation_[1]);
  ret.push_back(translation_[2]);
  ret.push_back(rotation_.w);
  ret.push_back(rotation_.v[0]);
  ret.push_back(rotation_.v[1]);
  ret.push_back(rotation_.v[2]);
  return ret;

}

Pose::Pose(const ci::Matrix44f &t) : translation_(t.getTranslate().xyz()) {

  ci::Matrix33f r = t.subMatrix33(0, 0);// .transposed(); //cinder is column major and ocv is row
  //cv::Mat rotation(3, 3, CV_32FC1, &r.m );
  
  //rotation_ = sv::Quaternion(rotation);
  rotation_ = ci::Quatf(r);

}

std::vector<ci::Vec3f> Pose::ComputeJacobian(const ci::Vec3f &point_) const {

  ci::Matrix44f self_pose = *this;
  
  ci::Vec3f point = self_pose.inverted() * point_;

  std::vector<ci::Vec3f> data(7);

  //translation dofs
  data[0] = ci::Vec3f(1.0f, 0.0f, 0.0f);
  data[1] = ci::Vec3f(0.0f, 1.0f, 0.0f);
  data[2] = ci::Vec3f(0.0f, 0.0f, 1.0f);

  ci::Vec3f r_point = rotation_ * point;
  ci::Matrix33f Cx; Cx.setToNull();
  Cx.at(0, 1) = -r_point[2]; Cx.at(0, 2) = r_point[1];
  Cx.at(1, 0) = r_point[2]; Cx.at(1, 2) = -r_point[0];
  Cx.at(2, 0) = -r_point[1]; Cx.at(2, 1) = r_point[0];
 
  Cx = Cx.transposed(); 
  
  data[3] = ci::Vec3f(0, 0, 0);
  data[4] = 2 * ci::Vec3f(Cx.at(0, 0), Cx.at(1, 0), Cx.at(2, 0));
  data[5] = 2 * ci::Vec3f(Cx.at(0, 1), Cx.at(1, 1), Cx.at(2, 1));
  data[6] = 2 * ci::Vec3f(Cx.at(0, 2), Cx.at(1, 2), Cx.at(2, 2));

  //return data;

  //rotation dofs - Qw
  data[3] = ci::Vec3f(
    (2.0f * (float)rotation_.v[1] * point[2]) - (2.0f * (float)rotation_.v[2] * point[1]),
    (2.0f * (float)rotation_.v[2] * point[0]) - (2.0f * (float)rotation_.v[0] * point[2]),
    (2.0f * (float)rotation_.v[0] * point[1]) - (2.0f * (float)rotation_.v[1] * point[0])
    );                                                                        
                                                                              
  // Qx                                                                       
  data[4] = ci::Vec3f(                                                        
    (2.0f * (float)rotation_.v[1] * point[1]) + (2.0f * (float)rotation_.v[2] * point[2]),
    (2.0f * (float)rotation_.v[1] * point[0]) - (4.0f * (float)rotation_.v[0] * point[1]) - (2.0f * (float)rotation_.w * point[2]),
    (2.0f * (float)rotation_.v[2] * point[0]) + (2.0f * (float)rotation_.w * point[1]) - (4.0f * (float)rotation_.v[0] * point[2])
    );                                                                                                                   
                                                                                                                         
  // Qy                                                                                                                  
  data[5] = ci::Vec3f(                                                                                                   
    (2.0f * (float)rotation_.v[0] * point[1]) - (4.0f * (float)rotation_.v[1] * point[0]) + (2.0f * (float)rotation_.w * point[2]),
    (2.0f * (float)rotation_.v[0] * point[0]) + (2.0f * (float)rotation_.v[2] * point[2]),                                 
    (2.0f * (float)rotation_.v[2] * point[1]) - (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.v[1] * point[2])
    );                                                                                                                   
                                                                                                                         
  // Qz  ==> previously [1] was (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.v[0] * point[1]) + (2.0f * (float)rotation_.v[1] * point[2]),                                                                                           
  data[6] = ci::Vec3f(                                                                                                   
    (2.0f * (float)rotation_.v[0] * point[2]) - (2.0f * (float)rotation_.w * point[1]) - (4.0f * (float)rotation_.v[2] * point[0]),
    (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.v[2] * point[1]) + (2.0f * (float)rotation_.v[1] * point[2]),
    (2.0f * (float)rotation_.v[0] * point[0]) + (2.0f * (float)rotation_.v[1] * point[1])
    );
  //translation dofs
  //data[0] = 1.0;
  //data[1] = 0.0;
  //data[2] = 0.0;
  //data[3] = 0.0;
  //data[4] = 1.0;
  //data[5] = 0.0;
  //data[6] = 0.0;
  //data[7] = 0.0;
  //data[8] = 1.0;

  ////rotation dofs
  //data[9] = (2 * rotation_.Y()*point[2]) - (2 * rotation_.v[2]*point[1]);
  //data[10] = (2 * rotation_.v[2]*point[0]) - (2 * rotation_.X()*point[2]);
  //data[11] = (2 * rotation_.X()*point[1]) - (2 * rotation_.Y()*point[0]);
  //data[12] = (2 * rotation_.Y()*point[1]) + (2 * rotation_.v[2]*point[2]);
  //data[13] = (2 * rotation_.Y()*point[0]) - (4 * rotation_.X()*point[1]) - (2 * rotation_.w*point[2]);
  //data[14] = (2 * rotation_.v[2]*point[0]) + (2 * rotation_.w*point[1]) - (4 * rotation_.X()*point[2]);
  //data[15] = (2 * rotation_.X()*point[1]) - (4 * rotation_.Y()*point[0]) + (2 * rotation_.w*point[2]);
  //data[16] = (2 * rotation_.X()*point[0]) + (2 * rotation_.v[2]*point[2]);
  //data[17] = (2 * rotation_.v[2]*point[1]) - (2 * rotation_.w*point[0]) - (4 * rotation_.Y()*point[2]);
  //data[18] = (2 * rotation_.X()*point[2]) - (2 * rotation_.w*point[1]) - (4 * rotation_.v[2]*point[0]);
  //data[19] = (2 * rotation_.w*point[0]) - (4 * rotation_.X()*point[1]) + (2 * rotation_.Y()*point[2]);
  //data[20] = (2 * rotation_.X()*point[0]) + (2 * rotation_.Y()*point[1]);

  return data;
}

void Pose::UpdatePose(const std::vector<float> &updates) {

  assert(updates.size() == GetNumDofs());

  for (int i = 0; i < 3; ++i)
    translation_[i] += updates[i];

  //w,x,y,z
  ci::Quatf rotation_update(updates[3], updates[4], updates[5], updates[6]);

  rotation_ = rotation_ + rotation_update;
  rotation_.normalize();
}
