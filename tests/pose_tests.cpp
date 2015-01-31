//#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "../include/ttrack/track/pose.hpp"
#include <opencv2/opencv.hpp>

template<typename T>
bool operator==(const ci::Matrix33<T> &a, const cv::Mat &b){
  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      if( std::abs(a.at(r,c)-b.at<T>(r,c)) > 0.01 ) return false;
    }
  }
  return true;
}

bool operator==(const cv::Mat &b, const ci::Matrix33d &a){
  return a==b;
}

BOOST_AUTO_TEST_SUITE( pose_test_suite )

BOOST_AUTO_TEST_CASE( pose_test ) {

  //cv::Mat rotation;
  //cv::Rodrigues(cv::Vec3f(5,0.45,-1),rotation);
  
  //ttrk::Pose pose(sv::Quaternion(rotation), ci::Vec3f(20, 15, 20));
  //ci::Matrix44f v1 = pose;
  
  //BOOST_ASSERT( v1.subMatrix33(0,0) == rotation );

  
}


BOOST_AUTO_TEST_SUITE_END()
