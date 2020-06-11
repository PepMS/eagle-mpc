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

class TrajectoryGeneratorDerived : public multicopter_mpc::TrajectoryGenerator {
 public:
  TrajectoryGeneratorDerived(const boost::shared_ptr<pinocchio::Model>& model,
                             const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params,
                             const double& dt, const boost::shared_ptr<multicopter_mpc::Mission>& mission)
      : multicopter_mpc::TrajectoryGenerator(model, mc_params, dt, mission) {}

  ~TrajectoryGeneratorDerived(){};

  const std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>&
  getDifferentialRunningModels() {
    return diff_models_running_;
  }
  const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>& getIntegratedRunningModels() {
    return int_models_running_;
  }
  const boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>& getDifferentialTerminalModel() {
    return diff_model_terminal_;
  }
  const boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>& getIntegratedTerminalModel() {
    return int_model_terminal_;
  }

  const boost::shared_ptr<crocoddyl::SolverAbstract>& getSolver() { return solver_; }
};

class TrajectoryGeneratorTest {
 public:
  TrajectoryGeneratorTest() {
    std::string mission_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/takeoff.yaml";
    std::string multirotor_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/iris.yaml";

    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                                pinocchio::JointModelFreeFlyer(), model_);

    yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
    yaml_parser::ParamsServer server_params(yaml_file.getParams());

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(server_params);

    yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
    yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

    mission_ = boost::make_shared<multicopter_mpc::Mission>(model_.nq + model_.nv);
    mission_->fillWaypoints(server_mission);
    mission_->fillInitialState(server_mission);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

    dt_ = 1e-2;
    trajectory_generator_ = boost::make_shared<TrajectoryGeneratorDerived>(mc_model_, mc_params_, dt_, mission_);
  }

  ~TrajectoryGeneratorTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;
  boost::shared_ptr<multicopter_mpc::Mission> mission_;

  double dt_;
  boost::shared_ptr<TrajectoryGeneratorDerived> trajectory_generator_;
};

BOOST_AUTO_TEST_CASE(ocp_constructor_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  // Base class constructor tests
  BOOST_CHECK(tg_test.mc_model_ == tg_test.trajectory_generator_->getModel());
  BOOST_CHECK(tg_test.mc_params_ == tg_test.trajectory_generator_->getMcParams());
  BOOST_CHECK(tg_test.dt_ == tg_test.trajectory_generator_->getTimeStep());
  BOOST_CHECK(tg_test.trajectory_generator_->getState() != nullptr);
  BOOST_CHECK(tg_test.trajectory_generator_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(tg_test.trajectory_generator_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(tg_test.trajectory_generator_->getActuation()->get_nu());
  tau_lb.head(tg_test.mc_params_->n_rotors_).fill(tg_test.mc_params_->min_thrust_);
  tau_ub.head(tg_test.mc_params_->n_rotors_).fill(tg_test.mc_params_->max_thrust_);
  BOOST_CHECK(tg_test.mc_params_->n_rotors_ == tg_test.trajectory_generator_->getActuationLowerBounds().size());
  BOOST_CHECK(tg_test.mc_params_->n_rotors_ == tg_test.trajectory_generator_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == tg_test.trajectory_generator_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == tg_test.trajectory_generator_->getActuationUpperBounds());
  BOOST_CHECK(tg_test.trajectory_generator_->getState()->zero() == tg_test.trajectory_generator_->getInitialState());

  BOOST_CHECK(tg_test.mc_model_->getFrameId(tg_test.mc_params_->base_link_name_) ==
              tg_test.trajectory_generator_->getBaseLinkId());

  // Derived class constructor test
  BOOST_CHECK(tg_test.mission_.get() == tg_test.trajectory_generator_->getMission().get());
  BOOST_CHECK(tg_test.trajectory_generator_->getKnots() == tg_test.mission_->getTotalKnots());
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1000.0);

  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_position == w_position);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_running == 1e-6);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_control_running == 1e-4);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_pos_running == 1e-2);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_vel_running == 1e-2);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_pos_terminal == 100);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_vel_terminal == 10);
}

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  std::string params_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/trajectory-generator-test.yaml";
  yaml_parser::ParserYAML yaml_file(params_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  tg_test.trajectory_generator_->loadParameters(server_params);

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_position == w_position);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_state_running == 1.234);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_control_running == 2.345);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_pos_running == 3.456);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_vel_running == 4.567);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_pos_terminal == 5.678);
  BOOST_CHECK(tg_test.trajectory_generator_->getParams().w_vel_terminal == 6.789);
}

BOOST_AUTO_TEST_CASE(trajectory_create_problem_action_models_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  
}

BOOST_AUTO_TEST_SUITE_END()