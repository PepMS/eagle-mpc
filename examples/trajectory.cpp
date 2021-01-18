#include <iostream>

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/parser_yaml.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/path.h"

int main(void) {
  boost::shared_ptr<multicopter_mpc::Trajectory> trajectory = multicopter_mpc::Trajectory::create();

  multicopter_mpc::ParserYaml parser("hover.yaml", "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory");
  multicopter_mpc::ParamsServer server(parser.get_params());

  trajectory->autoSetup(server);
}