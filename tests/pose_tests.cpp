//#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "../include/track/pose.hpp"
#include <opencv2/opencv.hpp>

template<typename T>
bool operator==(const ci::Matrix33<T> &a, const cv::Mat &b){
  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      if( a.at(r,c) != b.at<T>(r,c) ) return false;
    }
  }
  return true;
}

bool operator==(const cv::Mat &b, const ci::Matrix33d &a){
  return a==b;
}

BOOST_AUTO_TEST_SUITE( pose_test_suite )

BOOST_AUTO_TEST_CASE( pose_test ) {

  cv::Mat rotation;
  cv::Rodrigues(cv::Vec3d(5,0.45,-1),rotation);
  
  ttrk::Pose pose(cv::Vec3d(20,15,20),sv::Quaternion(rotation));
  ci::Matrix44d v1 = pose.AsCiMatrixForOpenGL();
  
  BOOST_ASSERT( v1.subMatrix33(0,0) == rotation );

  
}


BOOST_AUTO_TEST_SUITE_END()
