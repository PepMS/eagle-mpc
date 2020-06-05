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

class LowLevelControllerDerived : public multicopter_mpc::LowLevelController {
 public:
  LowLevelControllerDerived(const boost::shared_ptr<pinocchio::Model>& model,
                            const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params,
                            const double& dt, std::size_t& n_knots)
      : multicopter_mpc::LowLevelController(model, mc_params, dt, n_knots) {}

  ~LowLevelControllerDerived(){};

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
    low_level_controller_ = boost::make_shared<LowLevelControllerDerived>(mc_model_, mc_params_, dt_, n_knots_);
  }

  ~LowLevelControllerTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<LowLevelControllerDerived> low_level_controller_;
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

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
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

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  BOOST_CHECK(llc_test.low_level_controller_->getDifferentialRunningModels().size() ==
              llc_test.low_level_controller_->getKnots() - 1);
  BOOST_CHECK(llc_test.low_level_controller_->getIntegratedRunningModels().size() ==
              llc_test.low_level_controller_->getKnots() - 1);
  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    // Check that the data differential model that std_vecto<integrated> is pointing to is the same that the data
    // member differential is pointing to
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            llc_test.low_level_controller_->getIntegratedRunningModels()[i]);
    BOOST_CHECK(int_model->get_differential() == llc_test.low_level_controller_->getDifferentialRunningModels()[i]);
    // Check the action model ptr of every node are pointing to different action model
    if (i < llc_test.low_level_controller_->getKnots() - 2) {
      BOOST_CHECK(llc_test.low_level_controller_->getDifferentialRunningModels()[i] !=
                  llc_test.low_level_controller_->getDifferentialRunningModels()[i + 1]);
      BOOST_CHECK(llc_test.low_level_controller_->getIntegratedRunningModels()[i] !=
                  llc_test.low_level_controller_->getIntegratedRunningModels()[i + 1]);
    } else {
      BOOST_CHECK(llc_test.low_level_controller_->getDifferentialRunningModels()[i] !=
                  llc_test.low_level_controller_->getDifferentialTerminalModel());
      BOOST_CHECK(llc_test.low_level_controller_->getIntegratedRunningModels()[i] !=
                  llc_test.low_level_controller_->getIntegratedTerminalModel());
    }
  }
  BOOST_CHECK(llc_test.low_level_controller_->getIntegratedTerminalModel()->get_differential() ==
              llc_test.low_level_controller_->getDifferentialTerminalModel());
}

BOOST_AUTO_TEST_CASE(create_costs_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    BOOST_CHECK(llc_test.low_level_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("state_error")
                    ->second->weight == llc_test.low_level_controller_->getParams().w_state);
    BOOST_CHECK(llc_test.low_level_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("control")
                    ->second->weight == llc_test.low_level_controller_->getParams().w_control);
  }
  BOOST_CHECK(llc_test.low_level_controller_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("state_error")
                  ->second->weight == llc_test.low_level_controller_->getParams().w_state);
  BOOST_CHECK(llc_test.low_level_controller_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("control")
                  ->second->weight == llc_test.low_level_controller_->getParams().w_control);
}

BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
        boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
            llc_test.low_level_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("state_error")
                ->second->cost->get_activation());
    Eigen::VectorXd weights(llc_test.low_level_controller_->getParams().w_state_position.size() +
                            llc_test.low_level_controller_->getParams().w_state_orientation.size() +
                            llc_test.low_level_controller_->getParams().w_state_velocity_lin.size() +
                            llc_test.low_level_controller_->getParams().w_state_velocity_ang.size());
    weights << llc_test.low_level_controller_->getParams().w_state_position,
        llc_test.low_level_controller_->getParams().w_state_orientation,
        llc_test.low_level_controller_->getParams().w_state_velocity_lin,
        llc_test.low_level_controller_->getParams().w_state_velocity_ang;
    BOOST_CHECK(activation->get_weights() == weights);
  }
}

// Set reference before creating the problem
BOOST_AUTO_TEST_CASE(set_reference_test_1, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  std::vector<Eigen::VectorXd> reference_trajectory(llc_test.low_level_controller_->getKnots(),
                                                    llc_test.low_level_controller_->getState()->zero());
  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    reference_trajectory[i](0) = i;
  }

  llc_test.low_level_controller_->setReferenceStateTrajectory(reference_trajectory);
  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        llc_test.low_level_controller_->getDifferentialRunningModels()[i]
            ->get_costs()
            ->get_costs()
            .find("state_error")
            ->second->cost);
    BOOST_CHECK(reference_trajectory[i] == llc_test.low_level_controller_->getStateRef()[i]);
    BOOST_CHECK(cost_state->get_xref() == llc_test.low_level_controller_->getStateRef()[i]);
  }
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      llc_test.low_level_controller_->getDifferentialTerminalModel()
          ->get_costs()
          ->get_costs()
          .find("state_error")
          ->second->cost);
  BOOST_CHECK(reference_trajectory[llc_test.low_level_controller_->getKnots() - 1] ==
              llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
  BOOST_CHECK(cost_state->get_xref() ==
              llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
}

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(set_reference_test_2, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::vector<Eigen::VectorXd> reference_trajectory(llc_test.low_level_controller_->getKnots(),
                                                    llc_test.low_level_controller_->getState()->zero());

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    reference_trajectory[i](0) = i;
  }

  llc_test.low_level_controller_->setReferenceStateTrajectory(reference_trajectory);

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        llc_test.low_level_controller_->getDifferentialRunningModels()[i]
            ->get_costs()
            ->get_costs()
            .find("state_error")
            ->second->cost);
    BOOST_CHECK(reference_trajectory[i] == llc_test.low_level_controller_->getStateRef()[i]);
    BOOST_CHECK(cost_state->get_xref() == llc_test.low_level_controller_->getStateRef()[i]);
  }
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      llc_test.low_level_controller_->getDifferentialTerminalModel()
          ->get_costs()
          ->get_costs()
          .find("state_error")
          ->second->cost);
  BOOST_CHECK(reference_trajectory[llc_test.low_level_controller_->getKnots() - 1] ==
              llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
  BOOST_CHECK(cost_state->get_xref() ==
              llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
}

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(update_reference_trajectory_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::vector<Eigen::VectorXd> reference_trajectory(llc_test.low_level_controller_->getKnots(),
                                                    llc_test.low_level_controller_->getState()->zero());

  for (std::size_t i = 0; i < llc_test.low_level_controller_->getKnots() - 1; ++i) {
    reference_trajectory[i](0) = i;
  }

  llc_test.low_level_controller_->setReferenceStateTrajectory(reference_trajectory);
  Eigen::VectorXd state = llc_test.low_level_controller_->getState()->zero();
  state(4) = 0.4563;
  llc_test.low_level_controller_->updateReferenceStateTrajectory(state);

  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      llc_test.low_level_controller_->getDifferentialTerminalModel()
          ->get_costs()
          ->get_costs()
          .find("state_error")
          ->second->cost);
  BOOST_CHECK(state == llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
  BOOST_CHECK(cost_state->get_xref() ==
              llc_test.low_level_controller_->getStateRef()[llc_test.low_level_controller_->getKnots() - 1]);
}

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(set_solver_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  BOOST_CHECK(llc_test.low_level_controller_->getSolver() != nullptr);
}

BOOST_AUTO_TEST_CASE(solve_test, *boost::unit_test::tolerance(1e-7)) {
  LowLevelControllerTest llc_test;

  llc_test.low_level_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  Eigen::VectorXd reference = llc_test.low_level_controller_->getState()->zero();
  reference(2) = 0.3;
  std::vector<Eigen::VectorXd> reference_trajectory(llc_test.low_level_controller_->getKnots(), reference);
  llc_test.low_level_controller_->setReferenceStateTrajectory(reference_trajectory);

  llc_test.low_level_controller_->setInitialState(reference);
  BOOST_CHECK(llc_test.low_level_controller_->getInitialState() == reference);

  llc_test.low_level_controller_->solve();
  BOOST_CHECK(llc_test.low_level_controller_->getProblem()->get_x0() == reference);
  BOOST_CHECK(llc_test.low_level_controller_->getSolver()->get_stop() < 1e-7 );
}

BOOST_AUTO_TEST_SUITE_END()