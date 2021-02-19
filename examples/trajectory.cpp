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
#include "multicopter_mpc/sbfddp.hpp"

int main(void) {
  multicopter_mpc::ParserYaml parser_aux(
      "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/trajectory.yaml", "", true);
  multicopter_mpc::ParamsServer server_aux(parser_aux.get_params());
  std::string trajectory_yaml = server_aux.getParam<std::string>("trajectory_file");

  boost::shared_ptr<multicopter_mpc::Trajectory> trajectory = multicopter_mpc::Trajectory::create();
  multicopter_mpc::ParserYaml parser(trajectory_yaml,
                                     "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory");
  multicopter_mpc::ParamsServer server(parser.get_params());

  trajectory->autoSetup(server);

  // boost::shared_ptr<crocoddyl::ShootingProblem> problem =
  //     trajectory->createProblem(10, false, trajectory->get_robot_state()->zero(), "IntegratedActionModelEuler");
  // boost::shared_ptr<crocoddyl::SolverBoxFDDP> solver = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      trajectory->createProblem(10, true, "IntegratedActionModelEuler");
  boost::shared_ptr<multicopter_mpc::SolverSbFDDP> solver =
      boost::make_shared<multicopter_mpc::SolverSbFDDP>(problem, trajectory->get_squash());

  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  solver->setCallbacks(callbacks);

  solver->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR);

  for (std::size_t i = 0; i < trajectory->get_stages().size(); ++i) {
    for (auto cost = trajectory->get_stages()[i]->get_cost_types().begin();
         cost != trajectory->get_stages()[i]->get_cost_types().end(); ++cost) {
      std::cout << "Cost with name: " << cost->first << ". Enum type: " << int(cost->second) << std::endl;
    }
  }
}