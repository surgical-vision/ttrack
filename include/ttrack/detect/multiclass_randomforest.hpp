#ifndef _MULTICLASS_RANDOMFOREST_HPP_
#define _MULTICLASS_RANDOMFOREST_HPP_

#include "randomforest.hpp"

namespace ttrk{

  /**
  * @class MultiClassRandomForest
  * @brief The random forest classification.
  *
  * The class for handling classification with a random forest classifier. Can handle large numbers of classes.
  */

  class MultiClassRandomForest : public RandomForest {

  public:

    MultiClassRandomForest(const size_t num_classes) : num_classes_(num_classes) {}

    virtual bool ClassifyFrame(boost::shared_ptr<sv::Frame> frame) override;

  protected:

    size_t num_classes_;

  };




}

#endif
