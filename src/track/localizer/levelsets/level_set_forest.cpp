#define _USE_MATH_DEFINES
#include <cmath>

#include <ttrack/track/localizer/levelsets/level_set_forest.hpp>
#include <ttrack/utils/helpers.hpp>

#include <fstream>
#include <opencv2/imgproc/imgproc_c.h>

using namespace ttrk;

int ttrk::EllipticalFourierDescriptorBuilder::number_of_harmonics_ = 50;

// use the same number of components as the CompLS tracker

LevelSetForestTracker::LevelSetForestTracker(boost::shared_ptr<StereoCamera> camera) : ComponentLevelSet(3, camera) {

  dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/test_roll.dat") >> roll_predictor;
  dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/test_a1.dat") >> a1_predictor;
  dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/test_a2.dat") >> a2_predictor;
  dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/test_a3.dat") >> a3_predictor;

  //dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/mean.dat") >> mean_vector;
  //dlib::deserialize("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/var.dat") >> mean_vector;

}

LevelSetForestTracker::~LevelSetForestTracker() {


}

inline ci::Matrix44f MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation, const std::string &order) {

  float cosx = ci::math<float>::cos(xRotation);
  float cosy = ci::math<float>::cos(yRotation);
  float cosz = ci::math<float>::cos(zRotation);
  float sinx = ci::math<float>::sin(xRotation);
  float siny = ci::math<float>::sin(yRotation);
  float sinz = ci::math<float>::sin(zRotation);

  ci::Matrix33f xRotationMatrix; xRotationMatrix.setToIdentity();
  ci::Matrix33f yRotationMatrix; yRotationMatrix.setToIdentity();
  ci::Matrix33f zRotationMatrix; zRotationMatrix.setToIdentity();

  xRotationMatrix.at(1, 1) = xRotationMatrix.at(2, 2) = cosx;
  xRotationMatrix.at(1, 2) = -sinx;
  xRotationMatrix.at(2, 1) = sinx;

  yRotationMatrix.at(0, 0) = yRotationMatrix.at(2, 2) = cosy;
  yRotationMatrix.at(0, 2) = siny;
  yRotationMatrix.at(2, 0) = -siny;

  zRotationMatrix.at(0, 0) = zRotationMatrix.at(1, 1) = cosz;
  zRotationMatrix.at(0, 1) = -sinz;
  zRotationMatrix.at(1, 0) = sinz;

  ci::Matrix33f r;
  //xyz
  //ci::Matrix33f r = zRotationMatrix * yRotationMatrix * xRotationMatrix;

  //zyx
  if (order == "zyx")
    r = xRotationMatrix * yRotationMatrix * zRotationMatrix;
  else if (order == "xyz")
    r = zRotationMatrix * yRotationMatrix * xRotationMatrix;
  else if (order == "xzy")
    r = yRotationMatrix * zRotationMatrix * xRotationMatrix;
  else
    throw std::runtime_error("");

  ci::Matrix44f rr = r;
  rr.at(3, 3) = 1.0f;
  return rr;

}

void LevelSetForestTracker::TrackTargetInFrame(boost::shared_ptr<Model> current_model, boost::shared_ptr<sv::Frame> frame){

  frame_ = frame;


  if (curr_step >= NUM_STEPS || first_run_) {

    curr_step = 0;

  }

  ++curr_step;

  float error = DoAlignmentStep(current_model, true);

  //extract contour 

  std::vector<cv::Point> c = GetContourFromFrame(current_model, frame_->GetClassificationMapROI());
  if (c.size() > 10){

    //convert to FDs
    auto FDs = b.BuildFromContour(c);

    ci::app::console() << "Begin prediction" << std::endl;
    //classify
    std::vector<float> updates = Evaluate(FDs);
    
    ci::app::console() << "End prediction" << std::endl;

    ci::Vec3f eulers = GetZYXEulersFromQuaternion(current_model->GetBasePose().GetRotation());
    ci::Quatf new_rotation = MatrixFromIntrinsicEulers(eulers[2], eulers[1], updates[0], "zyx");

    std::vector<float> current_pose;
    current_model->GetPose(current_pose);
    std::vector<float> new_pose;
    for (size_t i = 0; i < 3; ++i)
      new_pose.push_back(current_pose[i]);
    new_pose.push_back(new_rotation.w);
    new_pose.push_back(new_rotation.v.x);
    new_pose.push_back(new_rotation.v.y);
    new_pose.push_back(new_rotation.v.z);

    new_pose.push_back(updates[1]);
    new_pose.push_back(updates[2]);
    new_pose.push_back(updates[3] / 2);
    new_pose.push_back(updates[3] / 2);

    current_model->SetPose(new_pose);
  }
 
  UpdateWithErrorValue(error);
  errors_.push_back(error);

}

std::vector<float> LevelSetForestTracker::Evaluate(const std::vector<ttrk::EllipticalFourierDescriptor> &desc){

  sample_type input;

  for (size_t i = 0; i < desc.size(); ++i){
    for (int j = 0; j < 4; ++j){

      input(i * 4 + j) = desc[i][j];

    } 
  }

  //input = input - mean_vector;
  //input = input - variance_vector;


  std::vector<float> pose_cpp;

  pose_cpp.push_back(roll_predictor(input));
  pose_cpp.push_back(a1_predictor(input));
  pose_cpp.push_back(a2_predictor(input));
  pose_cpp.push_back(a3_predictor(input));

  ci::app::console() << "Output : ";
  for (const auto &i : pose_cpp){
    ci::app::console() << i << " ";
  }
  ci::app::console() << std::endl;

  return pose_cpp;

}

std::vector<cv::Point> LevelSetForestTracker::GetContourFromFrame(boost::shared_ptr<Model> current_model, cv::Mat &frame){

  std::array<ci::Vec2i, 4> rectangle;
  cv::Mat affine_transform;
  GetSubWindowCoordinates(current_model, stereo_camera_->left_eye(), rectangle, affine_transform);

  cv::Mat view_frame = frame_->GetImageROI();

  cv::line(view_frame, toOpenCV(rectangle[0]), toOpenCV(rectangle[1]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[1]), toOpenCV(rectangle[2]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[2]), toOpenCV(rectangle[3]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[3]), toOpenCV(rectangle[0]), cv::Scalar(255, 0, 0), 3);

  const float width = std::sqrtf((rectangle[0].x - rectangle[1].x)*(rectangle[0].x - rectangle[1].x) + (rectangle[0].y - rectangle[1].y)*(rectangle[0].y - rectangle[1].y));
  const float height = std::sqrtf((rectangle[1].x - rectangle[2].x)*(rectangle[1].x - rectangle[2].x) + (rectangle[1].y - rectangle[2].y)*(rectangle[1].y - rectangle[2].y));

  std::vector<cv::Point2f> src_points;
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[0])));
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[1])));
  //src_points.push_back(cv::Point2f(toOpenCV(rectangle[2])));
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[3])));

  std::vector<cv::Point2f> dst_points;
  dst_points.push_back(cv::Point2f(0, 0));
  dst_points.push_back(cv::Point2f(width, 0));
  dst_points.push_back(cv::Point2f(0, height));
  //dst_points.push_back(cv::Point2f(width, height));

  affine_transform = cv::getAffineTransform(src_points, dst_points);

  cv::Mat output_frame = cv::Mat::zeros(frame.size(), CV_8UC1);
  for (int r = 0; r < output_frame.rows; ++r){
    for (int c = 0; c < output_frame.cols; ++c){

      cv::Vec<float, 5> re = frame.at < cv::Vec<float, 5> >(r, c);

      if (re[0] < re[1] || re[0] < re[2]){
        output_frame.at<unsigned char>(r, c) = 255;
      }
    }
  }

  cv::Mat subframe;
  cv::warpAffine(output_frame, subframe, affine_transform, cv::Size(width, height));

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(subframe, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  //frame = view_frame;
  std::vector<cv::Point> largest;
  for (size_t i = 0; i < contours.size(); ++i){
    if (contours[i].size() > largest.size()) largest = contours[i];
  }

  return largest;

}

void LevelSetForestTracker::GetSubWindowCoordinates(boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, std::array<ci::Vec2i, 4> &rectangle, cv::Mat &affine_transform) {

  const ci::Matrix44f pose = current_model->GetBasePose();
  ci::Quatf rotation = pose.subMatrix33(0, 0);
  ci::Vec3f eulers = GetXZYEulersFromQuaternion(rotation);
  const float x_rotation = eulers[0];

  const ci::Vec2i center_of_mass = camera->ProjectPointToPixel(pose * ci::Vec3f(0, 0, 0));
  const ci::Vec2i angle_of_shaft = camera->ProjectPointToPixel(pose * ci::Vec3f(0, 0, 10));

  ci::Vec2f local_vertical_axis = angle_of_shaft - center_of_mass; local_vertical_axis.normalize();
  ci::Vec2f local_horizontal_axis = ci::Vec2f(local_vertical_axis[1], -local_vertical_axis[0]);

  float distance = std::sqrtf((center_of_mass.x - angle_of_shaft.x)*(center_of_mass.x - angle_of_shaft.x) + (center_of_mass.y - angle_of_shaft.y)*(center_of_mass.y - angle_of_shaft.y));
  
  ci::Vec2i top_left = center_of_mass + (2.4 * distance*local_vertical_axis) + (2 * distance* local_horizontal_axis);
  ci::Vec2i top_right = center_of_mass + (2.4 * distance*local_vertical_axis) - (2 * distance* local_horizontal_axis);

  ci::Vec2i bottom_left = center_of_mass + (2 * distance* local_horizontal_axis);
  ci::Vec2i bottom_right = center_of_mass - (2 * distance* local_horizontal_axis);

  rectangle[0] = top_left;
  rectangle[1] = top_right;
  rectangle[2] = bottom_right;
  rectangle[3] = bottom_left;

  affine_transform = cv::Mat::eye(cv::Size(3, 2), CV_32FC1);
  float angle = acos(std::abs(local_horizontal_axis[1])); //minus as we're going back from subwindow to window coords
  affine_transform.at<float>(0, 0) = affine_transform.at<float>(0, 0) = cos(angle);
  affine_transform.at<float>(0, 1) = -sin(angle);
  affine_transform.at<float>(1, 0) = sin(angle);
  affine_transform.at<float>(0, 2) = bottom_left[0];
  affine_transform.at<float>(1, 2) = bottom_left[1];

}

std::vector<ttrk::EllipticalFourierDescriptor> ttrk::EllipticalFourierDescriptorBuilder::BuildFromImage(const std::string &filepath) const{

  cv::Mat im = cv::imread(filepath, 0);

  return BuildFromImage(im);

}

std::vector<ttrk::EllipticalFourierDescriptor> ttrk::EllipticalFourierDescriptorBuilder::BuildFromImage(const cv::Mat &im) const {

  std::vector<cv::Point> contour;

  /*
  std::ifstream ifs(filepath);
  while( !ifs.eof() ){
  int x,y;
  ifs >> x; ifs >> y;
  contour.push_back( cv::Point(x,y) );
  }
  return BuildFromContour(contour);
  */


  std::vector<std::vector<cv::Point> >contours;
  cv::findContours(im, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  if (contours.size() != 1) throw(std::runtime_error("Error, testing - should only be one contour!\n"));

  return BuildFromContour(contours[0]);

}

std::vector<ttrk::EllipticalFourierDescriptor> ttrk::EllipticalFourierDescriptorBuilder::BuildFromContour(const std::vector<cv::Point> &contour) const{

  std::vector<int> deltaX, deltaY;
  std::vector<double> deltaT, time;
  GetIncrementXYT(deltaX, deltaY, deltaT, time, contour);

  std::vector< ttrk::EllipticalFourierDescriptor > FSDs;

  double A0 = GetA0Coeff(deltaX, deltaT, time, contour[0].x);
  double C0 = GetC0Coeff(deltaY, deltaT, time, contour[0].y);

  FSDs.push_back(ttrk::EllipticalFourierDescriptor(A0, 0, C0, 0));

  auto ANCoeff = GetANCoeffs(deltaX, deltaT, time);
  auto BNCoeff = GetBNCoeffs(deltaX, deltaT, time);
  auto CNCoeff = GetCNCoeffs(deltaY, deltaT, time);
  auto DNCoeff = GetDNCoeffs(deltaY, deltaT, time);

  if (ANCoeff.size() != BNCoeff.size() && BNCoeff.size() != CNCoeff.size() && CNCoeff.size() != DNCoeff.size()) throw std::runtime_error("Error, vectors are not the same size!\n");

  for (size_t i = 0; i<ANCoeff.size(); ++i){

    FSDs.push_back(ttrk::EllipticalFourierDescriptor(ANCoeff[i], BNCoeff[i], CNCoeff[i], DNCoeff[i]));

  }


  double size, orientation;
  NormalizeAndGetSizeAndOrientation(FSDs, size, orientation);

  const cv::Point center = GetCenterOfContour(contour);

  //FSDs.insert(FSDs.begin(), ttrk::EllipticalFourierDescriptor(center.x, center.y, size, orientation));

  return FSDs;

}

std::vector<double> ttrk::EllipticalFourierDescriptorBuilder::ConvertDescriptorsToFeatureVector(const std::vector<EllipticalFourierDescriptor> &descriptor) const {

  std::vector<double> ret;

  for (auto desc = descriptor.begin(); desc != descriptor.end(); ++desc){
    for (size_t i = 0; i<desc->vals_.size(); ++i) ret.push_back(desc->vals_[i]);
  }

  return ret;

}



cv::Point ttrk::EllipticalFourierDescriptorBuilder::GetCenterOfContour(const std::vector<cv::Point> &contour) const {

  cv::Point ret(0, 0);

  for (auto pt = contour.begin(); pt != contour.end(); ++pt){
    ret += *pt;
  }
  ret.x = (int)((double)ret.x / contour.size());
  ret.y = (int)((double)ret.y / contour.size());

  return ret;

}


void ttrk::EllipticalFourierDescriptorBuilder::NormalizeAndGetSizeAndOrientation(std::vector<ttrk::EllipticalFourierDescriptor> &fds, double &size, double &orientation) const {

  double theta1 = 0.5 * atan(2 * (fds[1][0] * fds[1][1] + fds[1][2] * fds[1][3]) / ((fds[1][0] * fds[1][0]) + (fds[1][2] * fds[1][2]) - (fds[1][1] * fds[1][1]) - (fds[1][3] * fds[1][3])));

  std::vector<ttrk::EllipticalFourierDescriptor> star_fds;

  for (int i = 0; i<number_of_harmonics_; ++i){
    double A = cos(i * theta1) * fds[i][0] + sin(i * theta1) * fds[i][1];
    double B = -sin(i * theta1) * fds[i][0] + cos(i * theta1) * fds[i][1];
    double C = cos(i * theta1) * fds[i][2] + sin(i * theta1) * fds[i][3];
    double D = -sin(i * theta1) * fds[i][2] + cos(i * theta1) * fds[i][3];
    star_fds.push_back(ttrk::EllipticalFourierDescriptor(A, B, C, D));
  }


  double psi1 = atan(star_fds[1][2] / star_fds[1][0]);
  double semi_major = sqrt((star_fds[1][0] * star_fds[1][0]) + (star_fds[1][2] * star_fds[1][2]));

  //for(auto fd = fds.begin(); fd != fds.end() ; ++fd) *fd = ttrk::EllipticalFourierDescriptor( (*fd)[0]/semi_major, (*fd)[1]/semi_major, (*fd)[2]/semi_major, (*fd)[3]/semi_major ) ;
  for (size_t i = 0; i<fds.size(); ++i)
    fds[i] = ttrk::EllipticalFourierDescriptor(star_fds[i][0] / semi_major, star_fds[i][1] / semi_major, star_fds[i][2] / semi_major, star_fds[i][3] / semi_major);

  for (int i = 0; i<number_of_harmonics_; ++i){

    fds[i][0] = (cos(psi1) * star_fds[i][0] + sin(psi1) * star_fds[i][2]) / semi_major;
    fds[i][1] = (cos(psi1) * star_fds[i][1] + sin(psi1) * star_fds[i][3]) / semi_major;
    fds[i][2] = (-sin(psi1) * star_fds[i][0] + cos(psi1) * star_fds[i][2]) / semi_major;
    fds[i][3] = (-sin(psi1) * star_fds[i][1] + cos(psi1) * star_fds[i][3]) / semi_major;

  }

  size = semi_major;
  orientation = psi1;

}



double ttrk::EllipticalFourierDescriptorBuilder::GetA0Coeff(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time, const double contour_startx) const{

  //get A0 coefficient
  double sum1 = 0.0;
  for (size_t i = 1; i<deltaT.size(); ++i){
    double sum2 = 0.0;
    double sum3 = 0.0;
    double inner_diff = 0.0;

    for (size_t j = 1; j<i; ++j){
      sum2 += deltaX[j - 1];
      sum3 += deltaT[j - 1];
    }

    inner_diff = sum2 - ((double)deltaX[i - 1] / deltaT[i - 1])*sum3;
    sum1 += (((double)deltaX[i - 1] / (2 * deltaT[i - 1]))*((time[i] * time[i]) - (time[i - 1] * time[i - 1])) + inner_diff*(time[i] - time[i - 1]));
  }

  return ((1.0 / time.back()) * sum1) + contour_startx;

}

std::vector<double> ttrk::EllipticalFourierDescriptorBuilder::GetANCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j){
      sum1 += ((double)deltaX[j] / deltaT[j])*((cos(pi2n*i*time[j + 1] / time.back()) - cos(pi2n*i*time[j] / time.back())));
    }


    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);

  }

  return ret;
}
std::vector<double> ttrk::EllipticalFourierDescriptorBuilder::GetBNCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaX[j] / deltaT[j])*((sin(pi2n*i*time[j + 1] / time.back()) - sin(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;
}

double ttrk::EllipticalFourierDescriptorBuilder::GetC0Coeff(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const double contour_starty) const{

  //get A0 coefficient
  double sum1 = 0.0;
  for (size_t i = 1; i<deltaT.size(); ++i){
    double sum2 = 0.0;
    double sum3 = 0.0;
    double inner_diff = 0.0;

    for (size_t j = 1; j<i; ++j){
      sum2 += deltaY[j - 1];
      sum3 += deltaT[j - 1];
    }

    inner_diff = sum2 - ((double)deltaY[i - 1] / deltaT[i - 1])*sum3;
    sum1 += (((double)deltaY[i - 1] / (2 * deltaT[i - 1]))*((time[i] * time[i]) - (time[i - 1] * time[i - 1])) + inner_diff*(time[i] - time[i - 1]));
  }

  return ((1.0 / time.back()) * sum1) + contour_starty;

}


std::vector<double> ttrk::EllipticalFourierDescriptorBuilder::GetCNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaY[j] / deltaT[j])*((cos(pi2n*i*time[j + 1] / time.back()) - cos(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;

}

std::vector<double> ttrk::EllipticalFourierDescriptorBuilder::GetDNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaY[j] / deltaT[j])*((sin(pi2n*i*time[j + 1] / time.back()) - sin(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;

}

void ttrk::EllipticalFourierDescriptorBuilder::GetIncrementXYT(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const std::vector<cv::Point> &contour) const{

  for (size_t i = 1; i < contour.size(); ++i){
    deltaX.push_back(contour[i].x - contour[i - 1].x);
    deltaY.push_back(contour[i].y - contour[i - 1].y);
  }


  for (size_t i = 0; i< deltaX.size(); ++i)
    deltaT.push_back(std::sqrt(double(deltaX[i] * deltaX[i] + deltaY[i] * deltaY[i])));

  RemoveZeros(deltaX, deltaY, deltaT);

  time.push_back(0.0);
  for (size_t i = 1; i<deltaT.size(); ++i)
    time.push_back(time[i - 1] + deltaT[i - 1]);

}


void ttrk::EllipticalFourierDescriptorBuilder::RemoveZeros(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT) const{

  std::vector<int> new_deltaX, new_deltaY;
  std::vector<double> new_deltaT;

  for (size_t i = 0; i < deltaX.size(); ++i){

    if (deltaT[i] != 0.0){

      new_deltaX.push_back(deltaX[i]);
      new_deltaY.push_back(deltaY[i]);
      new_deltaT.push_back(deltaT[i]);

    }
  }

  deltaX = new_deltaX;
  deltaY = new_deltaY;
  deltaT = new_deltaT;

}