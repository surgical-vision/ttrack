#ifndef _DETECT_H_
#define _DETECT_H_

#include "headers.hpp"
#include "classifier.hpp"

namespace ttrk{

/**
 * @class Detect
 * @brief Detection system. Wraps the classification of images and interface with the TTrack
 * class.
 *
 * This is the class...
 */


  class Detect{
    
  public:

    Detect();
    ~Detect();

    /**
     * The operator overload for the boost thread call. This function wraps the calls to the detection methods.
     * @param[in] image The image pulled from the video file or image file.
     */

    void operator()(boost::shared_ptr<cv::Mat> image); 

    /**
     * The operator returns a unique pointer to the classified frame. 
     * @return a unique pointer to the classified frame.
     */
    boost::shared_ptr<cv::Mat> GetPtrToClassified() const;

    /**
     * Has the detector found a candidate for the object in the frame.
     * @return true if yes, false if no
     */
    bool Found() const;  

    /**
     * Setup the classifier 
     */
    void Setup(const std::string &root_dir);


  protected:
    
    std::string root_dir_; /**< A string containing the root directory where classifier, data etc is stored. */

    boost::shared_ptr<cv::Mat> c_frame_;
    boost::shared_ptr<cv::Mat> v_frame_;
    bool found_;
    Classifier classifier_;
  
  };

}

#endif
