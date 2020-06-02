#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

BOOST_AUTO_TEST_CASE(ocp_constructor_test, *boost::unit_test::tolerance(1e-7)) {
  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/takeoff.yaml";
  std::string multirotor_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/iris.yaml";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params =
      boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  mc_params->fill(server_params);

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  boost::shared_ptr<multicopter_mpc::Mission> mission =
      boost::make_shared<multicopter_mpc::Mission>(model.nq + model.nv);
  mission->fillWaypoints(server_mission);
  mission->fillInitialState(server_mission);

  boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);

  double dt = 1e-2;
  multicopter_mpc::TrajectoryGenerator trajectory(mc_model, mc_params, dt, mission);
  // Finish loading

  BOOST_CHECK(mc_model == trajectory.getModel());
  BOOST_CHECK(mc_params == trajectory.getMcParams());
  BOOST_CHECK(dt == trajectory.getTimeStep());
  BOOST_CHECK(trajectory.getState() != nullptr);
  BOOST_CHECK(trajectory.getActuation() != nullptr);

  Eigen::VectorXd tau_lb(trajectory.getActuation()->get_nu());
  Eigen::VectorXd tau_ub(trajectory.getActuation()->get_nu());
  tau_lb.head(mc_params->n_rotors_).fill(mc_params->min_thrust_);
  tau_ub.head(mc_params->n_rotors_).fill(mc_params->max_thrust_);
  BOOST_CHECK(mc_params->n_rotors_ == trajectory.getActuationLowerBounds().size());
  BOOST_CHECK(mc_params->n_rotors_ == trajectory.getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == trajectory.getActuationLowerBounds());
  BOOST_CHECK(tau_ub == trajectory.getActuationUpperBounds());
  BOOST_CHECK(trajectory.getState()->zero() == trajectory.getInitialState());

  BOOST_CHECK(mc_model->getFrameId(mc_params->base_link_name_) == trajectory.getBaseLinkId());
}

BOOST_AUTO_TEST_CASE(trajectory_constructor_test, *boost::unit_test::tolerance(1e-7)) {
  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/takeoff.yaml";
  std::string multirotor_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/iris.yaml";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params =
      boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  mc_params->fill(server_params);

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  boost::shared_ptr<multicopter_mpc::Mission> mission =
      boost::make_shared<multicopter_mpc::Mission>(model.nq + model.nv);
  mission->fillWaypoints(server_mission);
  mission->fillInitialState(server_mission);

  boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);

  double dt = 1e-2;
  multicopter_mpc::TrajectoryGenerator trajectory(mc_model, mc_params, dt, mission);
  // Finish loading
  BOOST_CHECK(mission.get() == trajectory.getMission().get());
}

BOOST_AUTO_TEST_CASE(trajectory_create_problem_action_models_test, *boost::unit_test::tolerance(1e-7)) {
  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/takeoff.yaml";
  std::string multirotor_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/iris.yaml";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params =
      boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  mc_params->fill(server_params);

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  boost::shared_ptr<multicopter_mpc::Mission> mission =
      boost::make_shared<multicopter_mpc::Mission>(model.nq + model.nv);
  mission->fillWaypoints(server_mission);
  mission->fillInitialState(server_mission);

  boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);

  double dt = 1e-2;
  multicopter_mpc::TrajectoryGenerator trajectory(mc_model, mc_params, dt, mission);
  // Finish loading
  multicopter_mpc::SolverTypes::Type solver_type = multicopter_mpc::SolverTypes::BoxFDDP;
  trajectory.createProblem(solver_type);
}

BOOST_AUTO_TEST_SUITE_END()