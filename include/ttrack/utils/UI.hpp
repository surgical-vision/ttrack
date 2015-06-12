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
      menubar_ = cinder::params::InterfaceGl(title, ci::Vec2i(width, height));
    }

    //std::vector<UIControllableVariableBase *> GetVars() { return vars_; }

    template<typename T>
    void AddVar(T *val, T min, T max, T step){
      std::stringstream ss;
      ss << "min = " << min << " max = " << max << " step = " + step;
      menubar_.addParam(v->GetName(), val, ss.str());
    }

    cinder::params::InterfaceGl &Menubar() { return menubar_; }

  protected:

    cinder::params::InterfaceGl menubar_;
    //std::vector<UIControllableVariableBase *> vars_;
    bool initialized_;


  private:

    UIController() { initialized_ = false; }

  };

  /*class UIControllableVariableBase {

  public:

    UIControllableVariableBase(const std::string &name) : name_(name) {}

    template<typename T>
    void RegisterVariable() {

      auto i = UIController::Instance();
      i.Add<T>(this);

    }

    virtual std::string GetName() const { return name_; }

    virtual void Increment() = 0;
    virtual void Decrement() = 0;
    virtual std::string GetIncrement() const = 0;
    virtual std::string GetVal() const = 0;
    virtual std::string GetMax() const = 0;
    virtual std::string GetMin() const = 0;
    virtual void *GetValPtr() = 0;
    inline virtual int GetValType() const = 0;

  protected:

    std::string name_;

  };

  template<typename T>
  class UIControllableVariable : public UIControllableVariableBase {

  public:

    UIControllableVariable(const T initial, const T increment, const T min, const T max, const std::string &name) : val_(initial), increment_(increment), min_(min), max_(max), UIControllableVariableBase(name) {
      RegisterVariable<T>();
    }

    operator T(){
      return val_;
    }

    operator T() const{
      return val_;
    }

    virtual void Increment() {
      val_ += increment_;
    }

    virtual void Decrement() {
      val_ -= increment_;
    }

    virtual std::string GetIncrement() const {
      std::stringstream ss;
      ss << increment_;
      return ss.str();
    }

    virtual std::string GetVal() const {
      std::stringstream ss;
      ss << val_;
      return ss.str();
    }

    virtual std::string GetMax() const {
      std::stringstream ss;
      ss << max_;
      return ss.str();
    }

    virtual std::string GetMin() const {
      std::stringstream ss;
      ss << min_;
      return ss.str();
    }

    virtual void *GetValPtr() {
      return (void *)&val_;
    }

    inline virtual int GetValType() const;

    T getValAsType() const {
      return val_;
    }

  private:

    T max_;
    T min_;
    T val_;
    T increment_;

  };

  template<>
  int UIControllableVariable<int>::GetValType() const {
    return 10;
  }

  template<>
  int UIControllableVariable<float>::GetValType() const {
    return 12;
  }

  template<>
  int UIControllableVariable<double>::GetValType() const {
    return 13;
  }*/
  
}