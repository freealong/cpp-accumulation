/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/9/3
 */

#pragma once

#include <iostream>
#include <string>
#include <map>

template <typename T>
class Factory {
 public:
  template <typename TDerived>
  void Register(std::string name) {
    static_assert(std::is_base_of<T, TDerived>::value,
                  "Factory::registerType doesn't accept this type because doesn't derive from base"
                  " class");
    create_funcs_[name] = &CreateFunc<TDerived>;
  }

  T* Create(std::string name) {
    auto it = create_funcs_.find(name);
    if (it != create_funcs_.end()) {
      return it->second();
    }
    return nullptr;
  }

 private:
  template <typename TDerived>
  static T* CreateFunc() {
    return new TDerived();
  }

  typedef T* (*PCreateFunc)();
  std::map<std::string, PCreateFunc> create_funcs_;
};
