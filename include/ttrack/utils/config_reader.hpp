#ifndef __CONFIG_READER_HPP__
#define __CONFIG_READER_HPP__

#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <sstream>

namespace ttrk {

  /**
  * @ConfigReader
  * @brief The handler for configuration files.
  * Loads and processes configuration files.
  */
  class ConfigReader {

  public:

    /**
    * Construct a config reader and process the file.
    * @param[in] config_file The path to the config file.
    */
    explicit ConfigReader(const std::string &config_file){

      std::ifstream ifs(config_file);
      if (!ifs.is_open()){
        throw std::runtime_error("Error, cannot open file!");
      }

      std::string line;
      while (std::getline(ifs, line))
      {
        if (line[0] == '#' || line.length() < 1) continue;
        remove_carriage_return(line);
        std::vector<std::string> tokens = split(line, '=');
        if (tokens.size() != 2) throw std::runtime_error("Error, bad parse!");
        config_[tokens[0]] = tokens[1];
      }
    }

    /**
    * Helper function to remove the carriage returns that are messing things up.
    * @param[in] line The line to clean up.
    */
    void remove_carriage_return(std::string& line) const {
      if (*line.rbegin() == '\r')
      {
        line.erase(line.length() - 1);
      }
    }

    /**
    * Get a value from the configuration file.
    * @param[in] key The key value to identify the element.
    * @return The element.
    */
    std::string get_element(const std::string &key) const {

      if (config_.count(key) == 0){
        throw std::runtime_error("Couldn't find key!");
      }
      else{
        std::map<std::string, std::string>::const_iterator it = config_.find(key);
        return it->second;
      }

    }

    /**
    * Helper to cast the element to type.
    * @param[in] key The key to identify the element.
    * @return The cast element.
    */
    template<typename T>
    T get_element_as_type(const std::string &key) const {

      if (config_.count(key) == 0){
        throw std::runtime_error("Couldn't find key!");
      }
      else{
        std::map<std::string, std::string>::const_iterator it = config_.find(key);
        std::stringstream s(it->second);
        T x;
        s >> x;
        return x;
      }

    }

  protected:

    /**
    * Helper to split a string on a delimiter.
    * @param[in] s The string to split.
    * @param[in] delim The char to split the string on.
    * @return The vector of segments.
    */
    std::vector<std::string> split(const std::string &s, char delim) const {
      std::vector<std::string> elems;
      split(s, delim, elems);
      return elems;
    }

    /**
    * Helper to split a string on a delimiter.
    * @param[in] s The string to split.
    * @param[in] delim The char to split the string on.
    * @param[out] elems The vector of segments.
    */
    void split(const std::string &s, char delim, std::vector<std::string> &elems) const {
      std::stringstream ss(s);
      std::string item;
      while (std::getline(ss, item, delim)) {
        elems.push_back(item);
      }
    }


    std::map<std::string, std::string> config_; /**< Mapping keys to their configuration values. */


  };



}

#endif