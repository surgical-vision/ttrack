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

    /**
    * Check if the loaded classifier supports classifying multiple classes or if it just does binary classification.
    * @return Whether the classifier supports mulitple classes or not.
    */
    virtual bool IsBinary() const override { return num_classes_ == 2; }

  protected:

    size_t num_classes_;

  };




}

#endif
