#include "../../headers/utils/camera.hpp"
#include "../../headers/utils/helpers.hpp"
#include <fstream>
using namespace ttrk;

MonocularCamera::MonocularCamera(const std::string &calibration_filename){

  cv::FileStorage fs;

  try{

    fs.open(calibration_filename,cv::FileStorage::READ); 
    fs["Camera_Matrix"] >> intrinsic_matrix_;
    fs["Distortion_Coefficients"] >> distortion_params_;


  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }



}

cv::Point2i MonocularCamera::ProjectPointToPixel(const cv::Point3f &point) const {
  cv::Point2f pt = ProjectPoint(point);
  //return cv::Point2i(ttrk::round(pt.x));
  return cv::Point2i(ttrk::round(pt.x),ttrk::round(pt.y));
}


cv::Point3f MonocularCamera::UnProjectPoint(const cv::Point2i &point) const {
  
  //cv::Point3f unprojected;
  cv::Mat projected(1,1,CV_32FC2);
  projected.at<cv::Vec2f>(0,0) = cv::Vec2f(point.x,point.y);
  cv::Mat unprojected(1,1,CV_32FC2);
  
  cv::undistortPoints(projected, unprojected, intrinsic_matrix_, distortion_params_); 

  return cv::Point3f(unprojected.at<cv::Vec2f>(0,0)[0],unprojected.at<cv::Vec2f>(0,0)[1],1);

}

cv::Point2f MonocularCamera::ProjectPoint(const cv::Point3f &point) const {

  std::vector<cv::Point2f> projected_point;
  static cv::Mat rot = cv::Mat::eye(3,3,CV_64FC1);
  static cv::Mat tran = cv::Mat::zeros(3,1,CV_64FC1);
  
  cv::projectPoints(std::vector<cv::Point3f>(1,point),rot,tran,intrinsic_matrix_,distortion_params_,projected_point);
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
    left_eye_.reset( new MonocularCamera(temp_intrinsic, temp_distortion) );
    rectified_left_eye_.reset( new MonocularCamera );

    fs["Right_Camera_Matrix"] >> temp_intrinsic;
    fs["Right_Distortion_Coefficients"] >> temp_distortion;
    right_eye_.reset( new MonocularCamera(temp_intrinsic, temp_distortion) );
    rectified_right_eye_.reset( new MonocularCamera );

    cv::Mat rotation(3,3,CV_64FC1),translation(3,1,CV_64FC1);
    fs["Extrinsic_Camera_Rotation"] >> rotation;
    fs["Extrinsic_Camera_Translation"] >> translation;
    for(int r=0;r<3;r++){
      for(int c=0;c<3;c++){
        extrinsic_matrix_.at<double>(r,c) = rotation.at<double>(r,c);
      }
      extrinsic_matrix_.at<double>(r,3) = translation.at<double>(r,0);
    }
    extrinsic_matrix_(cv::Range(3,4),cv::Range::all()) = 0.0;
    extrinsic_matrix_.at<double>(3,3) = 1.0;

  }catch(cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    SAFE_EXIT();

  }

}

void TestReproject(const cv::Mat &disparity_map, cv::Mat &reprojected_point_cloud, const cv::Mat &reprojection_matrix) {

  reprojected_point_cloud = cv::Mat(disparity_map.size(),CV_32FC3);

  for(int r=0;r<disparity_map.rows;r++){
    for(int c=0;c<disparity_map.cols;c++){

      cv::Vec3f &point = reprojected_point_cloud.at<cv::Vec3f>(r,c);

      double data[] = {r,c,disparity_map.at<float>(r,c),1.0};
      cv::Mat p(4,1,CV_64FC1,data);
      cv::Mat dp = reprojection_matrix * p;
      const double denom = dp.at<double>(3);
      point = cv::Vec3f( dp.at<double>(0)/denom, dp.at<double>(1)/denom, dp.at<double>(2)/denom);
      //if(data[2] == 14.0){
      //  std::cerr << cv::Point3f(point) << "\n";
      //}
    }
  }
  

}

void StereoCamera::ReprojectTo3D(const cv::Mat &disparity_image, cv::Mat &point_cloud, const std::vector<cv::Vec2i> &connected_region) const {

  if(point_cloud.data == 0x0) point_cloud.create(disparity_image.size(),CV_32FC3);
  cv::Mat disp_image = disparity_image.clone();

  //mask point cloud if required
  cv::Mat mask;
  if(connected_region.size() == 0){
    mask = cv::Mat::ones(disparity_image.size(),CV_8UC1) * 255;
  }else{
    mask = cv::Mat::zeros(disparity_image.size(),CV_8UC1);
  }
  unsigned char *mask_data = (unsigned char *)mask.data;
  const int cols = mask.cols;
  for(size_t i=0;i<connected_region.size();i++){
    const cv::Vec2i &pixel = connected_region[i];
    mask_data[pixel[1]*cols + pixel[0]] = 255;
  }

 

 /* for (int r = 0; r < disparity_image.rows; r++){
    for (int c =0;c < disparity_image.cols; c++ ){
      
      disp_image.at<int16_t>(r,c) *= (mask.at<unsigned char>(r,c) == 255);
      
    }
  }*/

  //cv::imwrite("mask.png",mask);
  //cv::reprojectImageTo3D(disp_image,point_cloud,reprojection_matrix_,true,-1);
  TestReproject(disp_image,point_cloud,reprojection_matrix_);

  //cv::Mat z = cv::Mat::zeros(disp_image.size(),CV_8UC1);
  //std::cerr << reprojection_matrix_ << "\n";

  //std::ofstream ofs("points.xyz");  
  for (int r = 0; r < point_cloud.rows; r++){
    for (int c =0; c < point_cloud.cols; c++ ){
      
        
      if(mask.at<unsigned char>(r,c) != 255)
        point_cloud.at<cv::Vec3f>(r,c) = cv::Vec3f(0,0,0) ;      
      
      if(point_cloud.at<cv::Vec3f>(r,c)[2] < 0 || point_cloud.at<cv::Vec3f>(r,c)[2] == 10000)
        point_cloud.at<cv::Vec3f>(r,c) = cv::Vec3f(0,0,0);
 
      cv::Point3f point(point_cloud.at<cv::Vec3f>(r,c) );
      if(point != cv::Point3f(0,0,0)){
        //std::cerr << disp_image.at<float>(r,c) << "-->" << point << "\n";
        //if(point.z < 0 || point.z > 100) point.z = 50;
        //ofs << point.x << " " << point.y << " " << point.z << "\n";
        //z.at<unsigned char>(r,c) = 255;
        
      }
     
    }
  }
  
  //cv::imwrite("negdisp.png",z);
  //ofs.close();


}

void StereoCamera::Rectify(const cv::Size image_size) {

  cv::stereoRectify(left_eye_->intrinsic_matrix_,left_eye_->distortion_params_,
                    right_eye_->intrinsic_matrix_,right_eye_->distortion_params_,
                    image_size,
                    extrinsic_matrix_(cv::Range(0,3),cv::Range(0,3)),
                    extrinsic_matrix_(cv::Range(0,3),cv::Range(3,4)),
                    R1, R2, P1, P2, reprojection_matrix_,
                    CV_CALIB_ZERO_DISPARITY, // 0 || CV_CALIB_ZERO_DISPARITY
                    0,  // -1 = default scaling, 0 = no black pixels, 1 = no source pixels lost
                    cv::Size(), &roi1, &roi2); 

  InitRectified();
  
  //store ROI1/2 in the stereo image class and then write method to extract these roi's whenever
  //useful image area methods are needed
  cv::initUndistortRectifyMap(left_eye_->intrinsic_matrix_,
      left_eye_->distortion_params_,
      R1,P1,image_size,CV_32F,mapx_left_,mapy_left_); //must be 16s or 32f

  cv::initUndistortRectifyMap(right_eye_->intrinsic_matrix_,
      right_eye_->distortion_params_,
      R2,P2,image_size,CV_32F,mapx_right_,mapy_right_);


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
