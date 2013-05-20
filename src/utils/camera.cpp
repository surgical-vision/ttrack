#include "../../headers/utils/camera.hpp"
#include "../../headers/utils/helpers.hpp"

using namespace ttrk;

MonocularCamera::MonocularCamera(const std::string &calibration_filename){

  cv::FileStorage fs;

  try{

    fs.open(calibration_filename,cv::FileStorage::READ); 
    fs["camera-intrinsic"] >> intrinsic_matrix_;
    fs["camera-distortion"] >> distortion_params_;


  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }



}

cv::Point2i MonocularCamera::ProjectPointToPixel(const cv::Point3f &point) const {
  cv::Point2f pt = ProjectPoint(point);
  return cv::Point2i(ttrk::round(pt.y),ttrk::round(pt.x));
}


cv::Point3f MonocularCamera::UnProjectPoint(const cv::Point2i &point) const {

  //cv::Point3f unprojected;
  cv::Mat unprojected;
  //cv::undistortPoints(cv::Mat(point,false),cv::Mat(unprojected,false), intrinsic_matrix_, distortion_params_); 
  cv::undistortPoints((cv::Mat)point, unprojected, intrinsic_matrix_, distortion_params_); 
  
  return (cv::Point3f)unprojected;

}

cv::Point2f MonocularCamera::ProjectPoint(const cv::Point3f &point) const {

  std::vector<cv::Point2f> projected_point;
  
  cv::projectPoints(std::vector<cv::Point3f>(1,point),cv::Mat::zeros(3,1,CV_64FC1),cv::Mat::zeros(3,1,CV_64FC1),intrinsic_matrix_,distortion_params_,projected_point);
  if(projected_point.size() != 1) throw(std::runtime_error("Error, projected points size != 1.\n"));

  return projected_point.front();

}


StereoCamera::StereoCamera(const std::string &calibration_filename){

  cv::FileStorage fs;

  try{

    cv::Mat temp_intrinsic, temp_distortion;

    fs.open(calibration_filename,cv::FileStorage::READ); 
    
    fs["left-camera-intrinsic"] >> temp_intrinsic;
    fs["left-camera-distortion"] >> temp_distortion;
    left_eye_ = MonocularCamera(temp_intrinsic, temp_distortion);

    fs["right-camera-intrinsic"] >> temp_intrinsic;
    fs["right-camera-distortion"] >> temp_distortion;
    right_eye_ = MonocularCamera(temp_intrinsic, temp_distortion);

    fs["extrinsic"] >> extrinsic_matrix_;


  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }



}
