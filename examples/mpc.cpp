#include <iostream>

#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/mpc-controllers/carrot-mpc.hpp"
#include "multicopter_mpc/utils/parser_yaml.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/path.h"
#include "multicopter_mpc/sbfddp.hpp"

int main(void) {
  multicopter_mpc::ParserYaml parser("mpc.yaml",
                                     "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc");
  multicopter_mpc::ParamsServer server(parser.get_params());

  // multicopter_mpc::CarrotMpc carrot_mpc()
}