///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_UTILS_LOG_HPP_
#define EAGLE_MPC_UTILS_LOG_HPP_

#include <iostream>

namespace eagle_mpc {

class Log {
 public:
  Log(const std::string &funcName) { std::cout << funcName; }

  template <class T>
  Log &operator<<(const T &v) {
    std::cout << v;
    return *this;
  }

  ~Log() {
    std::cout << "\033[0m" << std::endl;
  }
};

#define MMPC_INFO Log("\033[0;36m[EAGLE_MPC INFO]: ") // 36==cyan
#define MMPC_WARN Log("\033[0;33m[EAGLE_MPC WARN]:") // 33==yellow
#define MMPC_ERROR Log("\033[0;31m[EAGLE_MPC ERROR]") // 31==red

}  // namespace eagle_mpc
#endif