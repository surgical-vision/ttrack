#include "../../include/track/pose.hpp"

using namespace ttrk;

Pose::operator ci::Matrix44f() const {

  cv::Vec3f euler = rotation_.EulerAngles();
  ci::Matrix44f ret = ci::Matrix44f::createRotation(ci::Vec3d(euler[0],euler[1],euler[2]));
    
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
  
  for (int i = 0; i<3; i++) ret.at(i, 3) = translation_[i];
    
  return ret;

}

Pose::Pose(const ci::Matrix44f &t) : translation_(t.getTranslate().xyz()) {

  ci::Matrix33f r = t.subMatrix33(0, 0).transposed(); //cinder is column major and ocv is row
  cv::Mat rotation(3, 3, CV_64FC1, &r.m );
  
  rotation_ = sv::Quaternion(rotation);

}

std::vector<ci::Vec3f> Pose::ComputeJacobian(const ci::Vec3f &point) const {

  std::vector<ci::Vec3f> data(7);

  //translation dofs
  data[0] = ci::Vec3f(1.0f, 0.0f, 0.0f);
  data[1] = ci::Vec3f(0.0f, 1.0f, 0.0f);
  data[2] = ci::Vec3f(0.0f, 0.0f, 1.0f);

  //rotation dofs
  data[3] = ci::Vec3f(
    (2.0f * (float)rotation_.Y()*point[2]) - (2.0f * (float)rotation_.Z()*point[1]),
    (2.0f * (float)rotation_.Z()*point[0]) - (2.0f * (float)rotation_.X()*point[2]),
    (2.0f * (float)rotation_.X()*point[1]) - (2.0f * (float)rotation_.Y()*point[0])
    );

  data[4] = ci::Vec3f(
    (2.0f * (float)rotation_.Y()*point[1]) + (2.0f * (float)rotation_.Z()*point[2]),
    (2.0f * (float)rotation_.Y()*point[0]) - (4.0f * (float)rotation_.X()*point[1]) - (2.0f * (float)rotation_.W()*point[2]),
    (2.0f * (float)rotation_.Z()*point[0]) + (2.0f * (float)rotation_.W()*point[1]) - (4.0f * (float)rotation_.X()*point[2])
    );

  data[5] = ci::Vec3f(
    (2.0f * (float)rotation_.X()*point[1]) - (4.0f * (float)rotation_.Y()*point[0]) + (2.0f * (float)rotation_.W()*point[2]),
    (2.0f * (float)rotation_.X()*point[0]) + (2.0f * (float)rotation_.Z()*point[2]),
    (2.0f * (float)rotation_.Z()*point[1]) - (2.0f * (float)rotation_.W()*point[0]) - (4.0f * (float)rotation_.Y()*point[2])
    );

  data[6] = ci::Vec3f(
    (2.0f * (float)rotation_.X()*point[2]) - (2.0f * (float)rotation_.W()*point[1]) - (4.0f * (float)rotation_.Z()*point[0]),
    (2.0f * (float)rotation_.W()*point[0]) - (4.0f * (float)rotation_.X()*point[1]) + (2.0f * (float)rotation_.Y()*point[2]),
    (2.0f * (float)rotation_.X()*point[0]) + (2.0f * (float)rotation_.Y()*point[1])
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
  //data[9] = (2 * rotation_.Y()*point[2]) - (2 * rotation_.Z()*point[1]);
  //data[10] = (2 * rotation_.Z()*point[0]) - (2 * rotation_.X()*point[2]);
  //data[11] = (2 * rotation_.X()*point[1]) - (2 * rotation_.Y()*point[0]);
  //data[12] = (2 * rotation_.Y()*point[1]) + (2 * rotation_.Z()*point[2]);
  //data[13] = (2 * rotation_.Y()*point[0]) - (4 * rotation_.X()*point[1]) - (2 * rotation_.W()*point[2]);
  //data[14] = (2 * rotation_.Z()*point[0]) + (2 * rotation_.W()*point[1]) - (4 * rotation_.X()*point[2]);
  //data[15] = (2 * rotation_.X()*point[1]) - (4 * rotation_.Y()*point[0]) + (2 * rotation_.W()*point[2]);
  //data[16] = (2 * rotation_.X()*point[0]) + (2 * rotation_.Z()*point[2]);
  //data[17] = (2 * rotation_.Z()*point[1]) - (2 * rotation_.W()*point[0]) - (4 * rotation_.Y()*point[2]);
  //data[18] = (2 * rotation_.X()*point[2]) - (2 * rotation_.W()*point[1]) - (4 * rotation_.Z()*point[0]);
  //data[19] = (2 * rotation_.W()*point[0]) - (4 * rotation_.X()*point[1]) + (2 * rotation_.Y()*point[2]);
  //data[20] = (2 * rotation_.X()*point[0]) + (2 * rotation_.Y()*point[1]);

  return data;
}

void Pose::UpdatePose(const std::vector<float> &updates) {

  assert(updates.size() == GetNumDofs());

  for (int i = 0; i < 3; ++i)
    translation_[i] += updates[i];

  rotation_ = rotation_ + sv::Quaternion(updates[3], updates[4], updates[5], updates[6]);

}
