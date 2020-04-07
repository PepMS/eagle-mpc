#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.hpp"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/problem-mission.hpp"

int main(void) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  yaml_parser::ParserYAML yaml_file("../../config/multirotor/hector.yaml", "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  multicopter_mpc::MultiCopterBaseParams mc_params;
  mc_params.fill(server_params);

  yaml_parser::ParserYAML yaml_mission("../../config/mission/simple.yaml", "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  multicopter_mpc::Mission mission(model.nq + model.nv);
  mission.fillWaypoints(server_mission);
  mission.fillInitialState(server_mission);

  multicopter_mpc::ProblemMission multicopter_problem(
      boost::make_shared<multicopter_mpc::Mission>(mission),
      boost::make_shared<multicopter_mpc::MultiCopterBaseParams>(mc_params),
      boost::make_shared<pinocchio::Model>(model), model.getFrameId("base_link"), 1e-2);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem = multicopter_problem.createProblem();

  crocoddyl::SolverFDDP fddp(problem);
  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;
  cbs.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  fddp.setCallbacks(cbs);

  fddp.solve();
}