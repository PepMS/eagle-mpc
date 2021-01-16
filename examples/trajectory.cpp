#include <iostream>

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/path.h"

int main(void) {
  multicopter_mpc::Trajectory trajectory;
  std::string yaml_path = "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/hover.yaml";
  trajectory.loadStagesFromYaml(yaml_path);
}