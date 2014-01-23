//#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE model_test

#include <boost/test/unit_test.hpp>
#include "../include/track/model/articulated_model.hpp"

namespace ttrk {

  class TestArticulatedTool : public IntuitiveSurgicalLND {

  public:
    TestArticulatedTool(const std::string &s) : IntuitiveSurgicalLND(s) {}

    ci::Matrix44f getTransformationToEndNode(int child) const { 
      ArticulatedNode::Ptr root_node = articulated_model_->RootNode();
      return root_node->children_[0]->children_[child]->GetTransform();
    }

    int getNumberOfRootNodeChildren() {
      ArticulatedNode::Ptr root_node = articulated_model_->RootNode();
      return root_node->children_.size();
    }

    int getNumberOfFirstChildChildren() {
      ArticulatedNode::Ptr root_node = articulated_model_->RootNode();
      return root_node->children_[0]->children_.size();
    }

    int getNumberOfSecondChildChildren() {
      ArticulatedNode::Ptr root_node = articulated_model_->RootNode();
      int n = 0;
      for(auto child = root_node->children_[0]->children_.begin();
        child != root_node->children_[0]->children_.end(); ++child)
        n += (*child)->children_.size();
      return n;
    }


      


  };

}

BOOST_AUTO_TEST_CASE( tree_size_test ) {

  ttrk::TestArticulatedTool at("../../resources/intuitive_astree.json"); //load the json file

  BOOST_ASSERT( at.getNumberOfRootNodeChildren() == 1 );

  BOOST_ASSERT( at.getNumberOfFirstChildChildren() == 2 );

  BOOST_ASSERT( at.getNumberOfSecondChildChildren() == 0);

  //at.RotateHead(0.01);
  at.RotateClaspers(0.01,0.01);

  ci::Matrix44d end_node_left = at.getTransformationToEndNode(0);
  ci::Matrix44d end_node_right = at.getTransformationToEndNode(1);

  ci::Matrix44d test1;
  test1.setToIdentity();
  test1.at(0,3) = 0.0078226;
  test1.at(1,1) = 0.99995;
  test1.at(1,2) = -0.00999983;
  test1.at(1,3) = 18.1565;
  test1.at(2,1) = 0.00999983;
  test1.at(2,2) = 0.99995;
  test1.at(2,3) = -6.24917e-007;

  BOOST_ASSERT( test1 == end_node_left );
  BOOST_ASSERT( test1 == end_node_right );

}

