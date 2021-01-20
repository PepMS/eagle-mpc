#include <iostream>

#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/parser_yaml.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/path.h"

int main(void) {
  boost::shared_ptr<multicopter_mpc::Trajectory> trajectory = multicopter_mpc::Trajectory::create();

  multicopter_mpc::ParserYaml parser("hover.yaml", "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory");
  multicopter_mpc::ParamsServer server(parser.get_params());

  trajectory->autoSetup(server);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      trajectory->createProblem(10, false, trajectory->get_robot_state()->zero(), "IntegratedActionModelEuler");

  boost::shared_ptr<crocoddyl::SolverBoxFDDP> solver = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem);

  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  solver->setCallbacks(callbacks);

  solver->solve();
  std::cout << "Finished \n";
}