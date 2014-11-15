//#define BOOST_TEST_MAIN

#include "../include/track/model/articulated_model.hpp"
#include <boost/test/unit_test.hpp>


namespace ttrk {

  class TestArticulatedTool : public DenavitHartenbergArticulatedModel {

  public:
    TestArticulatedTool(const std::string &s) : DenavitHartenbergArticulatedModel(s) {}

    ci::Matrix44f getTransformationToEndNode(int child) const { 
      return ci::Matrix44f();
    }

    int getNumberOfChildren(size_t idx) const {
      return model_->GetChildByIdx(idx)->GetChildren().size();
    }

    int getNumberOfNodes(){

      int index = 0;
      while (true){
        const Node *idx = GetRootNode()->GetChildByIdx(index);
        if (idx == nullptr) break;
        index++;
      }
      return index;

    }

    boost::shared_ptr<const Node> GetRootNode() const {
      return model_;
    }


  };

}


BOOST_AUTO_TEST_SUITE( model_test_suite )

BOOST_AUTO_TEST_CASE( model_load_test ) {

  ttrk::TestArticulatedTool at("../../data/lnd/model/model.json"); //load the json file

  BOOST_ASSERT(at.getNumberOfChildren(0) == 1);

  BOOST_ASSERT(at.getNumberOfChildren(1) == 1);

  BOOST_ASSERT(at.getNumberOfChildren(2) == 1);

  BOOST_ASSERT(at.getNumberOfChildren(3) == 2);

  BOOST_ASSERT(at.getNumberOfChildren(5) == 0);

  BOOST_ASSERT(at.getNumberOfChildren(4) == 0);

 
}

BOOST_AUTO_TEST_SUITE_END()
