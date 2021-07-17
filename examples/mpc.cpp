// #include <iostream>

// #include "crocoddyl/core/optctrl/shooting.hpp"
// #include "crocoddyl/core/solvers/box-fddp.hpp"
// #include "crocoddyl/core/solvers/fddp.hpp"
// #include "crocoddyl/core/utils/callbacks.hpp"

// #include "crocoddyl/multibody/actuations/multicopter-base.hpp"
// #include "crocoddyl/multibody/states/multibody.hpp"

// #include "eagle_mpc/mpc-controllers/carrot-mpc.hpp"
// #include "eagle_mpc/utils/parser_yaml.hpp"
// #include "eagle_mpc/utils/params_server.hpp"
// #include "eagle_mpc/path.h"
// #include "eagle_mpc/sbfddp.hpp"

int main(void) {
//   eagle_mpc::ParserYaml parser_aux(
//       "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/trajectory.yaml", "", true);
//   eagle_mpc::ParamsServer server_aux(parser_aux.get_params());
//   std::string trajectory_yaml = server_aux.getParam<std::string>("trajectory_file");

//   boost::shared_ptr<eagle_mpc::Trajectory> trajectory = eagle_mpc::Trajectory::create();
//   trajectory->autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/" + trajectory_yaml);

//   // boost::shared_ptr<crocoddyl::ShootingProblem> problem =
//   //     trajectory->createProblem(10, false, trajectory->get_robot_state()->zero(), "IntegratedActionModelEuler");
//   // boost::shared_ptr<crocoddyl::SolverBoxFDDP> solver = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem);

//   boost::shared_ptr<crocoddyl::ShootingProblem> problem =
//       trajectory->createProblem(10, true, "IntegratedActionModelEuler");
//   boost::shared_ptr<eagle_mpc::SolverSbFDDP> solver =
//       boost::make_shared<eagle_mpc::SolverSbFDDP>(problem, trajectory->get_squash());

//   std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
//   callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
//   solver->setCallbacks(callbacks);

//   solver->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR);

//   eagle_mpc::CarrotMpc carrot_mpc(trajectory,
//                                         "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc/mpc.yaml");
}