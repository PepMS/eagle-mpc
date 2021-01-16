#ifndef MULTICOPTER_MPC_TRAJECTORY_HPP_
#define MULTICOPTER_MPC_TRAJECTORY_HPP_

#include <iostream>
#include <map>
#include <string>

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/stage.hpp"

namespace multicopter_mpc {
class Trajectory {
 public:
  Trajectory();
  ~Trajectory();

  void loadStagesFromYaml(const std::string& yaml_path);

 private:
 std::map<std::string, Stage> stages_;
};
}  // namespace multicopter_mpc

#endif