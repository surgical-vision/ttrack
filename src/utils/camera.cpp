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


StereoCamera::StereoCamera(const std::string &calibration_filename):rectified_(false),extrinsic_matrix_(4,4,CV_64FC1){

  cv::FileStorage fs;

  try{

    cv::Mat temp_intrinsic, temp_distortion;

    fs.open(calibration_filename,cv::FileStorage::READ); 

    fs["Left_Camera_Matrix"] >> temp_intrinsic;
    fs["Left_Distortion_Coefficients"] >> temp_distortion;
    left_eye_ = MonocularCamera(temp_intrinsic, temp_distortion);

    fs["Right_Camera_Matrix"] >> temp_intrinsic;
    fs["Right_Distortion_Coefficients"] >> temp_distortion;
    right_eye_ = MonocularCamera(temp_intrinsic, temp_distortion);
    
    fs["Extrinsic_Camera_Rotation"] >> extrinsic_matrix_(cv::Range(0,3),cv::Range(0,3));
    fs["Extrinsic_Camera_Translation"] >> extrinsic_matrix_(cv::Range(0,3),cv::Range(3,4));
    extrinsic_matrix_(cv::Range(3,4),cv::Range::all()) = 0.0;
    extrinsic_matrix_.at<double>(3,3) = 1.0;

  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }

}

void StereoCamera::Rectify(const cv::Size image_size) {

  cv::Mat P1,P2,R1,R2,Q;
 
  cv::stereoRectify(left_eye_.intrinsic_matrix_,left_eye_.distortion_params_,
                    right_eye_.intrinsic_matrix_,right_eye_.distortion_params_,
                    image_size,
                    extrinsic_matrix_(cv::Range(0,3),cv::Range(0,3)),
                    extrinsic_matrix_(cv::Range(0,3),cv::Range(3,4)),
                    R1, R2, P1, P2, Q,
                    0, // 0 || CV_CALIB_ZERO_DISPARITY
                    0,  // -1 = default scaling, 0 = no black pixels, 1 = no source pixels lost
                    cv::Size(), &roi1, &roi2); 

  cv::initUndistortRectifyMap(left_eye_.intrinsic_matrix_,
      left_eye_.distortion_params_,
      R1,P1,image_size,CV_32F,mapx_left_,mapy_left_); //must be 16s or 32f

  cv::initUndistortRectifyMap(right_eye_.intrinsic_matrix_,
      right_eye_.distortion_params_,
      R1,P1,image_size,CV_32F,mapx_right_,mapy_right_);


  rectified_ = true;

}

void StereoCamera::RemapLeftFrame(cv::Mat &image) const {

  cv::Mat rectified;
  cv::remap(image,rectified,mapx_left_,mapy_left_,CV_INTER_CUBIC);
  image = rectified;
  
}


void StereoCamera::RemapRightFrame(cv::Mat &image) const {

  cv::Mat rectified;
  cv::remap(image,rectified,mapx_right_,mapy_right_,CV_INTER_CUBIC);
  image = rectified;
  
}
