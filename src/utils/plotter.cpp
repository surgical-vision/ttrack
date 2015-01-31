#include "../../include/ttrack/utils/plotter.hpp"

using namespace ttrk;

void ErrorMetricPlottable::Register() const {

  ErrorMetricPlotter &emp = ErrorMetricPlotter::Instance();
  emp.RegisterPlotter(this);

}


ErrorMetricPlotter & ErrorMetricPlotter::Instance() {

  if (!constructed_){
    instance_.reset(new ErrorMetricPlotter());
    constructed_ = true;
  }
  return *(instance_.get());

}

void ErrorMetricPlotter::Destroy(){

  instance_.reset();
  constructed_ = false;

}

ErrorMetricPlotter::ErrorMetricPlotter() {}

ErrorMetricPlotter::ErrorMetricPlotter(const ErrorMetricPlotter &that){}

ErrorMetricPlotter &ErrorMetricPlotter::operator=(const ErrorMetricPlotter &that){

  // check for self-assignment
  if (this == &that) return *this;

  //if we aren't self-assigning then something is wrong.
  throw(std::runtime_error("Error, attempting to construct a new ErrorMetricPlotter.\n"));
  return *this;

}


bool ErrorMetricPlotter::constructed_ = false;

boost::scoped_ptr<ErrorMetricPlotter> ErrorMetricPlotter::instance_;