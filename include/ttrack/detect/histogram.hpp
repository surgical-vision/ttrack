#ifndef _HISTOGRAM_HPP_
#define _HISTOGRAM_HPP_

#include "baseclassifier.hpp"

namespace ttrk{

  const int histogram_channel_num = 3;
  const int histogram_bin_num = 32;

  template<int HISTOGRAM_CHANNEL_NUM, int HISTOGRAM_BIN_NUM>
  struct InternalHistogram {

    InternalHistogram() { Init(); }

    void Init()
    {
      for (int i = 0; i < HISTOGRAM_CHANNEL_NUM; ++i)
        std::fill(bins[i], bins[i] + HISTOGRAM_BIN_NUM, 0);
    }

    size_t const* operator[](size_t const i) const { return bins[i]; }
    size_t* operator[](size_t const i) { return bins[i]; }

  private:

    size_t bins[HISTOGRAM_CHANNEL_NUM][HISTOGRAM_BIN_NUM];

  };


  /**
  * @class Histogram
  * @brief The histogram classifier.
  *
  * The class for handling classification with a histogram. Can handle large numbers of classes in principal but for now just
  * does 3 classes. Will extend if have time.
  */
  class Histogram : public BaseClassifier {

  public:

    Histogram();

    virtual bool ClassifyFrame(boost::shared_ptr<sv::Frame> frame);

    /**
    * A function for training the classifier of choice. Will accept data in the form returned by the TrainData class.
    * @param[in] training_data The training data to be used for training.
    * @param[in] labels The class labels for each training sample.
    * @param[in] root_dir The root directory with the location where the classifier should be saved.
    */
    virtual void TrainClassifier(boost::shared_ptr<cv::Mat> training_data, boost::shared_ptr<cv::Mat> labels, boost::shared_ptr<std::string> root_dir);


    /**
    * A discrete prediction on the class a pixel belongs to.
    * @param[in] pixel The pixel from the NDImage.
    * @return The class label.
    */
    virtual size_t PredictClass(const cv::Mat &pixel) const;

    /**
    * A probabilistic prediction on the class a pixel belongs to.
    * @param[in] pixel The pixel from the NDImage.
    * @param[in] class_index The index of the class to find the probability of.
    * @return The probability of that class vs all others.
    */
    virtual float PredictProb(const cv::Mat &pixel, const size_t class_index) const;

    /**
    * A load function for loading the Histogram from an image pair.
    * @param[in] url The file url.
    */
    virtual void Load(const std::string &url);

    virtual void Save(const std::string &url) const;

    /**
    * Get the name of the random forest as a string: "random_forest".
    * This is useful for saving or loading the classifier from a directory.
    * @return The string "random_forest"
    */
    virtual std::string NameAsString() const;
    
    /**
    * Check if the loaded classifier supports classifying multiple classes or if it just does binary classification.
    * @return Whether the classifier supports mulitple classes or not.
    */
    virtual bool IsBinary() const override { return false; }

  protected:

    int num_classes_;

    float bin_divider_;
    size_t fg_area_;
    size_t bg_area_;
    
    InternalHistogram<histogram_channel_num, histogram_bin_num> foreground_tip_;
    InternalHistogram<histogram_channel_num, histogram_bin_num> foreground_shaft_;
    InternalHistogram<histogram_channel_num, histogram_bin_num> background_;

  };
}

#endif