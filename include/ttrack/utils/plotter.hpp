#pragma once

#include <boost/scoped_ptr.hpp>
#include <vector>
#include <string>

namespace ttrk{

  class ErrorMetricPlottable {

  public:

    ErrorMetricPlottable() : name_("") { Register();  }

    explicit ErrorMetricPlottable(const std::string &n) : name_(n) { Register(); }

    virtual ~ErrorMetricPlottable() {}

    virtual std::vector<float> GetErrorValues() const { return error_vals_; }

    virtual void UpdateWithErrorValue(const float error_value) { error_vals_.push_back(error_value); }

    virtual void SetName(const std::string &name) { name_ = name; }

  protected:

    void Register() const;

    std::string name_;
    std::vector<float> error_vals_;


  };

  class ErrorMetricPlotter {
  
  public:

    void RegisterPlotter(const ErrorMetricPlottable *emp){ to_plot_.push_back(emp);  }

    void Destroy();

    static ErrorMetricPlotter &Instance();

    std::vector<const ErrorMetricPlottable *> GetPlottables() const { return to_plot_; }

  protected:

    std::vector<const ErrorMetricPlottable *> to_plot_;

  private:

    ErrorMetricPlotter();
    ErrorMetricPlotter(const ErrorMetricPlotter &);
    ErrorMetricPlotter &operator=(const ErrorMetricPlotter &);
    static bool constructed_;
    static boost::scoped_ptr<ErrorMetricPlotter> instance_;

  };


}