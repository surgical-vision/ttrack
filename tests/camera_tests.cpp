#include "../include/utils/camera.hpp"
#include <boost/test/unit_test.hpp>

namespace ttrk {

  namespace test {

    class TestStereoCamera : public ttrk::StereoCamera {

    public:
      
      explicit TestStereoCamera(const std::string &calib_filename) : StereoCamera(calib_filename) {}

    };


  }

}

BOOST_AUTO_TEST_SUITE(camera_test_suite)

BOOST_AUTO_TEST_CASE(camera_load_test) {

  ttrk::test::TestStereoCamera camera("../../data/lnd2/camera/config.xml");
  const cv::Mat unprojected_points = camera.left_eye()->GetUnprojectedImagePlane(640,480);

  for (int r = 1; r < unprojected_points.rows-1; ++r){
    for (int c = 1; c < unprojected_points.cols-1; ++c){

      const cv::Vec2f point = unprojected_points.at<cv::Vec2f>(r, c); //(x,y)
      const cv::Point2d projected_point = camera.left_eye()->ProjectPoint(cv::Vec3d(point[0], point[1], 1));
      BOOST_CHECK_SMALL(projected_point.y - r, 0.08); //check project(unproject(x)) \approx x
      BOOST_CHECK_SMALL(projected_point.x - c, 0.08);

      const cv::Vec2f previous_r = unprojected_points.at<cv::Vec2f>(r - 1, c);
      BOOST_CHECK_GT(point[1], previous_r[1]); //check ordering is correct
          

      const cv::Vec2f previous_c = unprojected_points.at<cv::Vec2f>(r, c - 1);
      BOOST_CHECK_GT(point[0], previous_c[0]); //check 

    }
  }


}

BOOST_AUTO_TEST_SUITE_END()
