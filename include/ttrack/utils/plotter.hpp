#pragma once

#include <boost/scoped_ptr.hpp>
#include <vector>
#include <string>

namespace ttrk{

  /**
  * @class ErrorMetricPlottable
  * @brief A class that allows other classes to register as creating error values which can be draw to the UI. 
  */
  class ErrorMetricPlottable {

  public:

    /**
    * Create an error plottable with an empty name.
    */
    ErrorMetricPlottable() : name_("") { Register();  }

    /**
    * Create an error plottable with a specified name.
    * @param[in] n The name of the plottable.
    */
    explicit ErrorMetricPlottable(const std::string &n) : name_(n) { Register(); }

    /** 
    * Default destructor.
    */
    virtual ~ErrorMetricPlottable() {}

    /**
    * Get the error values for this plottable.
    * @return The error values.
    */
    virtual std::vector<float> GetErrorValues() const { return error_vals_; }

    /**
    * Add a new error value.
    * @param[in] error_value The newly computed error value.
    */
    virtual void UpdateWithErrorValue(const float error_value) { error_vals_.push_back(error_value); }

    /**
    * Set the name of the plottable class.
    * @param[in] name The name of the class.
    */
    virtual void SetName(const std::string &name) { name_ = name; }

  protected:

    /**
    * Register the class with ErrorMetricPlottable
    */
    void Register() const;

    std::string name_; /**< The name of the class to register. */
    std::vector<float> error_vals_; /**< The error values to plot. */


  };

  /**
  * @class ErrorMetricPlottable
  * @brief A singleton class that allows other classes to register as creating error values which can be draw to the UI.
  */
  class ErrorMetricPlotter {
  
  public:

    /**
    * Add a plottable object allowing it to register itself to draw on the graph.
    * @param[in] emp The plottable object.
    */
    void RegisterPlotter(const ErrorMetricPlottable *emp){ to_plot_.push_back(emp);  }

    /**
    * Destroy the singleton object.
    */
    void Destroy();

    /**
    * Get the instance or create it if it doesn't exist.
    * @return The singleton instance.
    */
    static ErrorMetricPlotter &Instance();

    /**
    * Get the plottables for drawing on the graph.
    * @return The plottables to draw.
    */
    std::vector<const ErrorMetricPlottable *> GetPlottables() const { return to_plot_; }

  protected:

    std::vector<const ErrorMetricPlottable *> to_plot_; /**< The plottable objects. */

  private:

    ErrorMetricPlotter();
    ErrorMetricPlotter(const ErrorMetricPlotter &);
    ErrorMetricPlotter &operator=(const ErrorMetricPlotter &);
    static bool constructed_;
    static boost::scoped_ptr<ErrorMetricPlotter> instance_;

  };


}