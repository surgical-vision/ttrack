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


    ci::Matrix44f GetPoseAtChild(size_t child) const {

      return model_->GetRelativeTransformToChild(child);

    }

    // 0.423999995
    // -0.495999992
    //-0.00400000019

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

  at.SetPose(std::vector<float>({ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.17822f, 0.052696f, 0.519177f / 2, -0.519177f / 2 }));

  ci::Matrix44f wrist_pitch(
    0.923925f, 1.67228e-008f, 0.382574f, 0.0f,
    0.382574f, -4.0386e-008f, -0.923925f, 0.0f,
    0.0f, 1.0f, -4.37114e-008f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f);

  ci::Matrix44f wrist_yaw(
    0.0486646f, 0.998612f, 0.0201507f, 0.0f,
    0.922642f, -0.0526715f, 0.382043f, 0.0f,
    0.382574f, -8.40974e-008f, -0.923925f, 0.0f,
    8.31532f, 1.50505e-007f, 3.44316f, 1.0f);

  ci::Matrix44f clasper_base(
    0.0486646f, 0.998612f, 0.0201507f, 0.0f,
    -0.382574f, 8.63998e-008f, 0.923925f, 0.0f,
    0.922642f, -0.0526715f, 0.382043f, 0.0f,
    8.31532f, 1.50505e-007f, 3.44316f, 1.0f);

  ci::Matrix44f clasper1(
    -0.189792f, 0.978674f, -0.0785883f, 0.0f,
    -0.382574f, 8.63998e-008f, 0.923925f, 0.0f,
    0.904221f, 0.20542f, 0.374415f, 0.0f,
    8.31532f, 1.50505e-007f, 3.44316f, 1.0f);

  ci::Matrix44f clasper2(
    0.283861f, 0.951634f, 0.117539f, 0.0f,
    -0.382574f, 8.63998e-008f, 0.923925f, 0.0f,
    0.879239f, -0.307233f, 0.36407f, 0.0f,
    8.31532f, 1.50505e-007f, 3.44316f, 1.0f);
  
  ci::Matrix44f wp = at.GetPoseAtChild(1);
  ci::Matrix44f wy = at.GetPoseAtChild(2);
  ci::Matrix44f cb = at.GetPoseAtChild(3);
  ci::Matrix44f c1 = at.GetPoseAtChild(4);
  ci::Matrix44f c2 = at.GetPoseAtChild(5);
 
  for (int i = 0; i < 4; ++i){
    for (int j = 0; j < 4; ++j){
      BOOST_CHECK_SMALL(std::abs(wp.at(i, j)-wrist_pitch.at(i, j)), 0.001f);
      BOOST_CHECK_SMALL(std::abs(wy.at(i, j)-wrist_yaw.at(i, j)), 0.001f);
      BOOST_CHECK_SMALL(std::abs(cb.at(i, j)-clasper_base.at(i, j)), 0.001f);
      BOOST_CHECK_SMALL(std::abs(c1.at(i, j)-clasper1.at(i, j)), 0.001f);
      BOOST_CHECK_SMALL(std::abs(c2.at(i, j)-clasper2.at(i, j)), 0.001f);
    }
  }

}

BOOST_AUTO_TEST_SUITE_END()
