#ifndef _EXCEPTIONS_HPP_
#define _EXCEPTIONS_HPP_
#include <stdexcept>

namespace ttrk{

  /**
  * @class FileSysErr
  * @brief A custom exception for specifing paricular errors that are thrown by the training system when looking for the training set.
  */
  class FileSysErr : public std::exception {

  public:

    /**
    * @enum The type of error.
    */
    enum Type { 
      NoFiles, /*!< The directory contains no files. */
      NoDir /*!< The directory does not exist. */
    };

    /** 
    * Construct a filesystem exception specifying which type of error caused it. 
    * @param[in] name_ The name of the directory causing the problem.
    * @param[in] type_ The type of error.
    */
    FileSysErr(const std::string &name_, Type type_) { name = name_; type = type_; } 
#ifdef __linux__
    ~FileSysErr() throw() {};
#endif

    std::string name; /**< The name of the directory causing the problem. */
    Type type; /**< The type of problem the directory encountered. */
  
  private:

    FileSysErr();

  };
}
#endif //_EXCEPTIONS_HPP_
