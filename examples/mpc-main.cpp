// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/multibody/model.hpp"

// #include "example-robot-data/path.hpp"

// #include "crocoddyl/core/solvers/ddp.hpp"

// #include "yaml_parser/parser_yaml.h"

// #include "multicopter_mpc/multicopter-base-params.hpp"
// #include "multicopter_mpc/mission.hpp"
// #include "multicopter_mpc/problem-mission.hpp"
// #include "multicopter_mpc/ocp/trajectory-generator.hpp"
// #include "multicopter_mpc/path.h"

#include "multicopter_mpc/mpc-main.hpp"

int main(void) {
  multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, multicopter_mpc::SolverTypes::BoxFDDP);
  // pinocchio::Model model;
  // pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
  //                             pinocchio::JointModelFreeFlyer(), model);

  // yaml_parser::ParserYAML yaml_file(MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml", "", true);
  // yaml_parser::ParamsServer server_params(yaml_file.getParams());

  // boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params =
  //     boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  // mc_params->fill(server_params);

  // yaml_parser::ParserYAML yaml_mission(MULTICOPTER_MPC_MISSION_DIR "/passthrough.yaml", "", true);
  // yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  // boost::shared_ptr<multicopter_mpc::Mission> mission =
  //     boost::make_shared<multicopter_mpc::Mission>(model.nq + model.nv);
  // mission->fillWaypoints(server_mission);
  // mission->fillInitialState(server_mission);

  // boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);

  // multicopter_mpc::TrajectoryGenerator trajectory(mc_model, mc_params, 1e-2, mission);

  // trajectory.createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  // trajectory.setSolverCallbacks(true);
  // trajectory.solve();
}