#pragma once

#include <string>
#include <vector>
#include <cinder/params/Params.h>

namespace ttrk{

  class UIControllableVariableBase;

  class UIController {

  public:
  
    static UIController &Instance(){
      static UIController instance;
      return instance;
    }

    bool IsInitialized() { return initialized_; }

    void Initialize(const std::string &title, int width, const int height){
      initialized_ = true;
      menubar_ = cinder::params::InterfaceGl::create(title, ci::Vec2i(width, height));
    }

    void AddFunction(const std::string &name, const std::function<void ()> &function){

      menubar_->addButton(name, function);

    }

    void AddSeparator(){

      static size_t index = 0;
      std::stringstream ss;
      ss << "separator" << index;
      index++;
      menubar_->addSeparator(ss.str());

    }

    template<typename T>
    void AddMemberFunction(const std::string &name, std::function<void> &function, T *self){

      menubar_->addButton(name, std::bind(function, self));

    }


    template<typename T>
    void AddVar(const std::string &name, T *val, T min, T max, T step){
      std::stringstream ss;
      ss << "min = " << min << " max = " << max << " step = " + step;
      menubar_->addParam(name, val, ss.str());
    }

    cinder::params::InterfaceGlRef Menubar() { return menubar_; }

  protected:

    cinder::params::InterfaceGlRef menubar_;
    bool initialized_;


  private:

    UIController() { initialized_ = false; }

  };
}