#ifndef MULTICOPTER_MPC_TRAJECTORY_HPP_
#define MULTICOPTER_MPC_TRAJECTORY_HPP_

#include <iostream>
#include <map>
#include <string>

#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/stage.hpp"

namespace multicopter_mpc {
class Trajectory {
 public:
  Trajectory();
  ~Trajectory();

  void autoSetup(const ParamsServer& server);

 private:
 std::map<std::string, Stage> stages_;
};
}  // namespace multicopter_mpc

#endif