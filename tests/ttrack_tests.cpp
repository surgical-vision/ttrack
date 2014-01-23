#define BOOST_TEST_MODULE ttrack_test

#include <boost/test/unit_test.hpp>
#include "../include/ttrack.hpp"

namespace ttrk {

  class TestTTrack : public TTrack {




  };

}



BOOST_AUTO_TEST_CASE ( ttrack_test_setup ) {

  const std::string root_dir = "../data/lnd";
  t_track = ttrk::TTrack::Instance();
  t_track->SetUp( root_dir + "/" + "model/model.json", root_dir + "/" + "camera/config.xml", root_dir + "/" + "classifier/config.xml", root_dir + "/" + "results/", ttrk::RF,ttrk::STEREO);

  //BOOST_ASSERT( boost::dynamic_ptr_cast<


}
