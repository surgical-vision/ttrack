#pragma once

#include "comp_ls.hpp"
#include <array>

#include <dlib/svm.h>

namespace ttrk {

  struct EllipticalFourierDescriptor {

    EllipticalFourierDescriptor(const double A, const double B, const double C, const double D) {
      vals_[0] = A; vals_[1] = B; vals_[2] = C; vals_[3] = D;
    }

    double operator[](int n) const { return vals_[n]; }
    double &operator[](int n) { return vals_[n]; }

    std::array<double, 4> vals_;

    friend bool operator==(const EllipticalFourierDescriptor &e1, const EllipticalFourierDescriptor &e2){
      for (size_t i = 0; i<e1.vals_.size(); ++i) if (e1.vals_[i] != e2.vals_[i]) return false;
      return true;
    }

  };

  class EllipticalFourierDescriptorBuilder {

  public:

    std::vector<EllipticalFourierDescriptor> BuildFromImage(const cv::Mat &image) const;
    std::vector<EllipticalFourierDescriptor> BuildFromImage(const std::string &filepath) const;
    std::vector<EllipticalFourierDescriptor> BuildFromContour(const std::vector<cv::Point> &contour) const;
    std::vector<double> ConvertDescriptorsToFeatureVector(const std::vector<EllipticalFourierDescriptor> &descriptor) const;

  protected:

    cv::Point GetCenterOfContour(const std::vector<cv::Point> &contour) const;
    void NormalizeAndGetSizeAndOrientation(std::vector<ttrk::EllipticalFourierDescriptor> &fds, double &size, double &orientation) const;

    double GetA0Coeff(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time, const double contour_startx) const;
    double GetC0Coeff(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const double contour_starty) const;

    std::vector<double> GetDNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const;
    std::vector<double> GetCNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const;
    std::vector<double> GetBNCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const;
    std::vector<double> GetANCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const;

    void GetIncrementXYT(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const std::vector<cv::Point> &contour) const;
    void RemoveZeros(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT) const;
    static int number_of_harmonics_;

  };

  class LevelSetForestTracker : public ComponentLevelSet {

  public:

    LevelSetForestTracker(boost::shared_ptr<StereoCamera> camera);

    virtual ~LevelSetForestTracker();

    virtual void TrackTargetInFrame(boost::shared_ptr<Model> model, boost::shared_ptr<sv::Frame> frame);

  protected:

    std::vector<float> Evaluate(const std::vector<ttrk::EllipticalFourierDescriptor> &desc);

    //float DoAlignmentStep(boost::shared_ptr<Model> current_model, bool track_points);

    void GetSubWindowCoordinates(boost::shared_ptr<Model> current_model, boost::shared_ptr<MonocularCamera> camera, std::array<ci::Vec2i, 4> &rectangle, cv::Mat &affine_transform);

    std::vector<cv::Point> GetContourFromFrame(boost::shared_ptr<Model> current_model, cv::Mat &frame);
    
    EllipticalFourierDescriptorBuilder b;


    typedef dlib::matrix<double, 200, 1> sample_type;
    typedef double label_type;

    typedef dlib::radial_basis_kernel<sample_type> kernel_type;
  
    dlib::decision_function<kernel_type> roll_predictor;
    dlib::decision_function<kernel_type> a1_predictor;
    dlib::decision_function<kernel_type> a2_predictor;
    dlib::decision_function<kernel_type> a3_predictor;

    sample_type mean_vector;
    sample_type variance_vector;

  };

}