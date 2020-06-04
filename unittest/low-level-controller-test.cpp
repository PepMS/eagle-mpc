#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/low-level-controller.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class LowLevelControllerTest {
 public:
  LowLevelControllerTest() {
    std::string multirotor_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/iris.yaml";

    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                                pinocchio::JointModelFreeFlyer(), model_);

    yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
    yaml_parser::ParamsServer server_params(yaml_file.getParams());

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(server_params);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

    dt_ = 1e-2;
    n_knots_ = 100;
    low_level_controller_ =
        boost::make_shared<multicopter_mpc::LowLevelController>(mc_model_, mc_params_, dt_, n_knots_);
  }

  ~LowLevelControllerTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<multicopter_mpc::LowLevelController> low_level_controller_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  // Ocp_Base constructor
  BOOST_CHECK(llc_test.mc_model_ == llc_test.low_level_controller_->getModel());
  BOOST_CHECK(llc_test.mc_params_ == llc_test.low_level_controller_->getMcParams());
  BOOST_CHECK(llc_test.dt_ == llc_test.low_level_controller_->getTimeStep());
  BOOST_CHECK(llc_test.low_level_controller_->getState() != nullptr);
  BOOST_CHECK(llc_test.low_level_controller_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(llc_test.low_level_controller_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(llc_test.low_level_controller_->getActuation()->get_nu());
  tau_lb.head(llc_test.mc_params_->n_rotors_).fill(llc_test.mc_params_->min_thrust_);
  tau_ub.head(llc_test.mc_params_->n_rotors_).fill(llc_test.mc_params_->max_thrust_);
  BOOST_CHECK(llc_test.mc_params_->n_rotors_ == llc_test.low_level_controller_->getActuationLowerBounds().size());
  BOOST_CHECK(llc_test.mc_params_->n_rotors_ == llc_test.low_level_controller_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == llc_test.low_level_controller_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == llc_test.low_level_controller_->getActuationUpperBounds());

  BOOST_CHECK(llc_test.low_level_controller_->getState()->zero() == llc_test.low_level_controller_->getInitialState());
  BOOST_CHECK(llc_test.mc_model_->getFrameId(llc_test.mc_params_->base_link_name_) ==
              llc_test.low_level_controller_->getBaseLinkId());

  // Low Level constructor
  BOOST_CHECK(llc_test.n_knots_ == llc_test.low_level_controller_->getKnots());
  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots(); ++i) {
    BOOST_CHECK(llc_test.low_level_controller_->getState()->zero() ==
                llc_test.low_level_controller_->getStateRef()[i]);
  }
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  std::string params_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/low-level-controller-test.yaml";
  yaml_parser::ParserYAML yaml_file(params_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  llc_test.low_level_controller_->loadParameters(server_params);

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state == 4.5e-3);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_position == w_position);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_control == 2.3e-4);
}

BOOST_AUTO_TEST_CASE(initialize_load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1.0);

  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state == 1e-2);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_position == w_position);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(llc_test.low_level_controller_->getParams().w_control == 1e-4);
}

BOOST_AUTO_TEST_SUITE_END()