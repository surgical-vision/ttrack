#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cinder/gl/gl.h>
#include <cinder/Camera.h>
#include <cinder/app/AppBasic.h>
#include <cinder/gl/Light.h>

#include "../../include/utils/camera.hpp"
#include "../../include/constants.hpp"
#include "../../include/utils/helpers.hpp"

using namespace ttrk;

MonocularCamera::MonocularCamera(const std::string &calibration_filename){

  cv::FileStorage fs;

  try{

    cv::Mat cam_matrix;
    fs.open(calibration_filename,cv::FileStorage::READ); 
    fs["Camera_Matrix"] >> cam_matrix;

    fx_ = cam_matrix.at<double>(0, 0);
    fy_ = cam_matrix.at<double>(1, 1);
    px_ = cam_matrix.at<double>(0, 2);
    py_ = cam_matrix.at<double>(1, 2);

    fs["Distortion_Coefficients"] >> distortion_params_;

    cv::Mat image_dims;
    fs["Image_Dimensions"] >> image_dims;
    image_width_ = image_dims.at<int>(0);
    image_height_ = image_dims.at<int>(1);
    
  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }

}

MonocularCamera::MonocularCamera(const cv::Mat &intrinsic, const cv::Mat &distortion, const int image_width, const int image_height) :distortion_params_(distortion), image_width_(image_width), image_height_(image_height){

  fx_ = intrinsic.at<double>(0, 0);
  fy_ = intrinsic.at<double>(1, 1);
  px_ = intrinsic.at<double>(0, 2);
  py_ = intrinsic.at<double>(1, 2);

}

cv::Mat MonocularCamera::CameraMatrix() const { 

  cv::Mat cm = cv::Mat::eye(3, 3, CV_32FC1);  
  cm.at<float>(0, 0) = fx_;
  cm.at<float>(1, 1) = fy_;
  cm.at<float>(0, 2) = px_;
  cm.at<float>(1, 2) = py_;
  return cm;

}

cv::Point2i MonocularCamera::ProjectPointToPixel(const cv::Point3d &point) const {
  cv::Point2d pt = ProjectPoint(point);
  return cv::Point2i(ttrk::round(pt.x),ttrk::round(pt.y));
}

cv::Mat MonocularCamera::GetUnprojectedImagePlane(const int width, const int height) {

  if (unprojected_image_.data != nullptr) {
    return unprojected_image_;
  }
  else{
    std::vector<cv::Vec2f> points,outpoints;
    points.reserve(width*height);
    for (int r = 0; r < height; ++r){
      for (int c = 0; c < width; ++c){
        points.push_back(cv::Vec2f(c, r));
      }
    }

    cv::undistortPoints(points, outpoints, CameraMatrix(), distortion_params_);

    unprojected_image_ = cv::Mat(height, width, CV_32FC2);
    float *data = (float *)unprojected_image_.data;
    const int channels = 2;
    for (int r = 0; r < height; ++r){
      for (int c = 0; c < width; ++c){
        const int index = ((width*r) + c);
        data[index*channels] = outpoints[index][0];
        data[(index*channels) + 1] = outpoints[index][1];
      }
    }
  
    return unprojected_image_;
  }


}

cv::Point3d MonocularCamera::UnProjectPoint(const cv::Point2i &point) const {
  
  cv::Mat projected(1,1,CV_64FC2);
  projected.at<cv::Vec2d>(0,0) = cv::Vec2d(point.x,point.y);
  cv::Mat unprojected(1,1,CV_64FC2);
  
  cv::undistortPoints(projected, unprojected, CameraMatrix(), distortion_params_);

  return cv::Point3d(unprojected.at<cv::Vec2d>(0,0)[0],unprojected.at<cv::Vec2d>(0,0)[1],1);

}

void MonocularCamera::SetupCameraForDrawing(const int viewport_width, const int viewport_height) const {
  
  glViewport(0, 0, viewport_width, viewport_height);

  ci::CameraPersp camP;

  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, -1);
  ci::Vec3f world_up(0, 1, 0);

  view_direction = rotation_ * view_direction;
  world_up = rotation_ * world_up;

  camP.setEyePoint(camera_center_);
  camP.setViewDirection(view_direction);
  camP.setWorldUp(world_up);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrixf(camP.getModelViewMatrix().m);

  //ci::CameraPersp cam;
  //cam.setEyePoint(ci::Vec3f(0, 0, 0));
  //cam.setViewDirection(ci::Vec3f(0, 0, -1));
  //cam.setWorldUp(ci::Vec3f(0, 1, 0));
 
  //glMatrixMode(GL_MODELVIEW);
  //glLoadIdentity();
  //glMultMatrixf(cam.getModelViewMatrix().m);

  SetGLProjectionMatrix();

}

void MonocularCamera::SetGLProjectionMatrix() const {

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  const int near_clip_distance = GL_NEAR;
  const int far_clip_distance = GL_FAR;

  ci::Matrix44f gl_projection_matrix_;


  gl_projection_matrix_.setToNull();
  gl_projection_matrix_.m00 = fx_;
  gl_projection_matrix_.m11 = fy_;
  gl_projection_matrix_.m02 = -px_;
  gl_projection_matrix_.m12 = (float)-(image_height_ - py_);
  gl_projection_matrix_.m22 = (float)(near_clip_distance + far_clip_distance);
  gl_projection_matrix_.m23 = (float)(near_clip_distance * far_clip_distance);
  gl_projection_matrix_.m32 = -1;

  glOrtho(0, image_width_, 0, image_height_, GL_NEAR, GL_FAR);
  glMultMatrixf(gl_projection_matrix_.m);

}

cv::Point2d MonocularCamera::ProjectPoint(const cv::Point3d &point) const {

  std::vector<cv::Point2d> projected_point;
  static cv::Mat rot = cv::Mat::eye(3,3,CV_64FC1);
  static cv::Mat tran = cv::Mat::zeros(3,1,CV_64FC1);
  
  cv::projectPoints(std::vector<cv::Point3d>(1,point),rot,tran,CameraMatrix(),distortion_params_,projected_point);
  if(projected_point.size() != 1) throw(std::runtime_error("Error, projected points size != 1.\n"));
  
  return projected_point.front();

}


StereoCamera::StereoCamera(const std::string &calibration_filename) {

  if(!boost::filesystem::exists(boost::filesystem::path(calibration_filename)))
    throw(std::runtime_error("Error, could not find camera calibration file: " + calibration_filename + "\n"));

  cv::FileStorage fs;  

  try{

    cv::Mat image_dims;
    cv::Mat l_intrinsic, l_distortion;
    cv::Mat r_intrinsic, r_distortion;
    fs.open(calibration_filename, cv::FileStorage::READ);

    fs["Image_Dimensions"] >> image_dims;
    int image_width = image_dims.at<int>(0);
    int image_height = image_dims.at<int>(1);

    fs["Left_Camera_Matrix"] >> l_intrinsic;
    fs["Left_Distortion_Coefficients"] >> l_distortion;
    left_eye_.reset(new MonocularCamera(l_intrinsic, l_distortion, image_width, image_height));

    fs["Right_Camera_Matrix"] >> r_intrinsic;
    fs["Right_Distortion_Coefficients"] >> r_distortion;
    right_eye_.reset(new MonocularCamera(r_intrinsic, r_distortion, image_width, image_height));

    cv::Mat rotation(3,3,CV_64FC1),translation(3,1,CV_64FC1);
    fs["Extrinsic_Camera_Rotation"] >> rotation;
   
    fs["Extrinsic_Camera_Translation"] >> translation;

    for(int r=0;r<3;r++){
      for(int c=0;c<3;c++){
        right_eye_->rotation_.at(r,c) = rotation.at<double>(r,c);
      }
      right_eye_->camera_center_[r] = translation.at<double>(r, 0);
    }

    left_eye_->rotation_.setToIdentity();
    left_eye_->camera_center_ = ci::Vec3f(0, 0, 0);

  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }

}


void StereoCamera::SetupGLCameraFromRight() const{

  right_eye_->SetGLProjectionMatrix();

  ci::CameraPersp camP;

  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, 1);
  ci::Vec3f world_up(0, -1, 0);

  view_direction = ciExtrinsicRotation() * view_direction;
  world_up = ciExtrinsicRotation() * world_up;

  camP.setEyePoint(ci::Vec3f(ciExtrinsicTranslation()));
  camP.setViewDirection(view_direction);
  camP.setWorldUp(world_up);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrixf(camP.getModelViewMatrix().m);


}

void StereoCamera::SetupGLCameraFromLeft() const{

  left_eye_->SetGLProjectionMatrix();

  ci::CameraPersp camP;

  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, 1);
  ci::Vec3f world_up(0, -1, 0);

  camP.setEyePoint(eye_point);
  camP.setViewDirection(view_direction);
  camP.setWorldUp(world_up);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrixf(camP.getModelViewMatrix().m);

}

cv::Mat StereoCamera::cvExtrinsicRotation() const{

  cv::Mat ret(3, 3, CV_32FC1);
  for (int r = 0; r < 3; ++r){
    for (int c = 0; c < 3; ++c){
      ret.at<float>(r, c) = right_eye_->rotation_.at(r, c);
    }
  }
  return ret;

}

cv::Vec3d StereoCamera::cvExtrinsicTranslation() const{

  cv::Vec3d v(right_eye_->camera_center_[0], right_eye_->camera_center_[1], right_eye_->camera_center_[2]);
  return v;

}