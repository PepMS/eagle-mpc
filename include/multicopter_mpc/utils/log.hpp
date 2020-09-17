#ifndef MULTICOPTER_MPC_UTILS_LOG_HPP_
#define MULTICOPTER_MPC_UTILS_LOG_HPP_

#include <iostream>

namespace multicopter_mpc {

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

#define MMPC_INFO Log("\033[0;36m[MULTICOPTER_MPC INFO]: ") // 36==cyan
#define MMPC_WARN Log("\033[0;33m[MULTICOPTER_MPC WARN]:") // 33==yellow
#define MMPC_ERROR Log("\033[0;31m[MULTICOPTER_MPC ERROR]") // 31==red

}  // namespace multicopter_mpc
#endif