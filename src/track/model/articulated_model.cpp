#include "../../../include/track/model/articulated_model.hpp"
#include "cinder/ObjLoader.h"
#include "cinder/Json.h"
using namespace ttrk;

inline void LoadMesh(ci::TriMesh *mesh, ci::JsonTree::ConstIter iter){

  ci::ObjLoader loader(ci::loadFile( (*iter)["file"].getValue<std::string>() ));
  loader.load( mesh );

}

void ArticulatedTool::ParseJsonTree(ci::JsonTree &jt, ArticulatedNode::Ptr node){

  for(auto child = jt.begin(); child != jt.end(); ++child){

    node->children_.push_back( ArticulatedNode::Ptr(new ArticulatedNode) );
    node->children_.back()->LoadData(*child,node);
    ParseJsonTree(*child,node->children_.back());

  }

}

void ArticulatedTool::LoadFromJsonFile(const std::string &json_file){

  try{

    ci::JsonTree loader(ci::loadFile(json_file));

    ParseJsonTree(loader,articulated_model_->RootNode());

  }catch(ci::Exception &e){

    

  }



}



ArticulatedTool::ArticulatedTool(const std::string &model_parameter_file) {

 
  const std::string ext = boost::filesystem::path(model_parameter_file).extension().string();

  if( ext == ".json" )
    LoadFromJsonFile(model_parameter_file);
  else
    throw(std::runtime_error("Error, supported file type.\n"));

  //for each child

 

    /*


    ArticulatedComponent ac;
    

    ac.name_ =  (*tree_iter)["name"].getValue<std::string>();

    ac.transform_.setToIdentity();
    ac.articulation_.setToIdentity();

    if( !(*tree_iter)["articulated"].getValue<bool>() ) {

      ac.movable_ = false;
      //ac.acp_ = models_.end();
      ac.rel_to = "";

    }else{

      ac.movable_ = true;
      ac.com_[0] = (*tree_iter)["center"]["x"].getValue<double>() ;
      ac.com_[1] = (*tree_iter)["center"]["y"].getValue<double>() ;
      ac.com_[2] = (*tree_iter)["center"]["z"].getValue<double>() ;

      ac.transform_.translate(ac.com_);

      ac.axis_[0] = (*tree_iter)["axis"]["x"].getValue<double>() ;
      ac.axis_[1] = (*tree_iter)["axis"]["y"].getValue<double>() ;
      ac.axis_[2] = (*tree_iter)["axis"]["z"].getValue<double>() ;

      ac.max_angle_ = (*tree_iter)["max_angle"].getValue<double>();
      ac.min_angle_ = (*tree_iter)["min_angle"].getValue<double>();

      ac.rel_to = (*tree_iter)["relative_to"].getValue<std::string>();

      //ac.acp_ = std::find(models_.begin(),models_.end(),(*tree_iter)["relative_to"].getValue<std::string>());


    }




    models_.push_back(ac);


  }

  for(auto model = models_.begin(); model != models_.end(); ++model){

    model->acp_ = std::find(models_.begin(),models_.end(),model->rel_to);

  }
  */

}


void IntuitiveSurgicalLND::RotateHead(const double angle) {



}


void IntuitiveSurgicalLND::RotateClaspers(const double angle_1,const double angle_2) {



}
