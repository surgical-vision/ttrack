//#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE model_test

#include <boost/test/unit_test.hpp>
#include "../include/track/model/articulated_model.hpp"

namespace ttrk {

  class TestArticulatedTool : public ArticulatedTool {

  public:
    TestArticulatedTool(const std::string &s) : ArticulatedTool(s) {}

    ci::Matrix44f getTransformationToEndNode(int child) const { 
      ArticulatedNode::Ptr root_node = articulated_model_->RootNode();
      return root_node->children_[0]->children_[child]->getTransform();
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

  ttrk::TestArticulatedTool at("somefile.json"); //load the json file

  BOOST_ASSERT( at.getNumberOfRootNodeChildren() == 1 );

  BOOST_ASSERT( at.getNumberOfFirstChildChildren() == 2 );

  BOOST_ASSERT( at.getNumberOfSecondChildChildren() == 0);



  ci::Matrix44f end_node_left = at.getTransformationToEndNode(0);
  ci::Matrix44f end_node_right = at.getTransformationToEndNode(1);
}

/* after 0.01 rads rotations
clasper1 
 |           1           0           0   0.0078226|
 |           0     0.99995 -0.00999983     18.1565|
 |           0  0.00999983     0.99995-6.24917e-007|
 |           0           0           0           1|

clasper2 
 |           1           0           0   0.0078226|
 |           0     0.99995 -0.00999983     18.1565|
 |           0  0.00999983     0.99995-6.24917e-007|
 |           0           0           0           1|

 */
