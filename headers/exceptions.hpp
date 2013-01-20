#ifndef _EXCEPTIONS_HPP_
#define _EXCEPTIONS_HPP_
#include <stdexcept>

namespace ttrk{

  class FileSysErr : public std::exception {

  public:

    enum Type { NoFiles, NoDir };

    FileSysErr(const std::string &name_, Type type_) { name = name_; type = type_; } 
    std::string name;
    Type type;
  
  private:

    FileSysErr();

  };
}
#endif //_EXCEPTIONS_HPP_
