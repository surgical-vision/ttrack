#pragma once

#include "comp_ls.hpp"

namespace ttrk{

  class ArticulatedComponentLevelSet : public ComponentLevelSet {

    ArticulatedComponentLevelSet(size_t number_of_components, boost::shared_ptr<StereoCamera> camera) : ComponentLevelSet(number_of_components, camera) {}

  };


}