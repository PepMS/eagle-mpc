#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/mpc/rail-mpc.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class RailMpcDerived : public multicopter_mpc::RailMpc {
 public:
  RailMpcDerived(const boost::shared_ptr<pinocchio::Model>& model,
                 const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params,
                 const boost::shared_ptr<multicopter_mpc::Mission>& mission)
      : multicopter_mpc::RailMpc(model, mc_params, mission) {}

  ~RailMpcDerived(){};

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
  const boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> getIntegratedTerminalModel() {
    return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(int_model_terminal_);
  }

  const boost::shared_ptr<crocoddyl::SolverDDP>& getSolver() { return solver_; }
};

class RailMpcTest {
 public:
  RailMpcTest(const std::string& mission_name) {
    std::string multirotor_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/multirotor/iris.yaml";
    std::string mission_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/mission/" + mission_name;

    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                                pinocchio::JointModelFreeFlyer(), model_);

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(multirotor_yaml_path);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

    mc_mission_ = boost::make_shared<multicopter_mpc::Mission>(mc_model_->nq + mc_model_->nv);
    mc_mission_->fillWaypoints(mission_yaml_path);

    dt_ = 1e-2;
    n_knots_ = 100;
    rail_mpc_ = boost::make_shared<RailMpcDerived>(mc_model_, mc_params_, mc_mission_);
    rail_mpc_->setTimeStep(dt_);
  }

  ~RailMpcTest() {}

  pinocchio::Model model_;
  boost::shared_ptr<multicopter_mpc::Mission> mc_mission_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<RailMpcDerived> rail_mpc_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  // Ocp_Base constructor
  BOOST_CHECK(llc_test.mc_model_ == llc_test.rail_mpc_->getModel());
  BOOST_CHECK(llc_test.mc_params_ == llc_test.rail_mpc_->getMcParams());
  BOOST_CHECK(llc_test.dt_ == llc_test.rail_mpc_->getTimeStep());
  BOOST_CHECK(llc_test.rail_mpc_->getStateMultibody() != nullptr);
  BOOST_CHECK(llc_test.rail_mpc_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(llc_test.rail_mpc_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(llc_test.rail_mpc_->getActuation()->get_nu());
  tau_lb.head(llc_test.mc_params_->n_rotors_).fill(llc_test.mc_params_->min_thrust_);
  tau_ub.head(llc_test.mc_params_->n_rotors_).fill(llc_test.mc_params_->max_thrust_);
  BOOST_CHECK(llc_test.mc_params_->n_rotors_ == llc_test.rail_mpc_->getActuationLowerBounds().size());
  BOOST_CHECK(llc_test.mc_params_->n_rotors_ == llc_test.rail_mpc_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == llc_test.rail_mpc_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == llc_test.rail_mpc_->getActuationUpperBounds());

  BOOST_CHECK(llc_test.rail_mpc_->getStateMultibody()->zero() == llc_test.rail_mpc_->getInitialState());
  BOOST_CHECK(llc_test.mc_model_->getFrameId(llc_test.mc_params_->base_link_name_) ==
              llc_test.rail_mpc_->getBaseLinkId());

  // Low Level constructor
  BOOST_CHECK(llc_test.n_knots_ == llc_test.rail_mpc_->getKnots());
  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots(); ++i) {
    BOOST_CHECK(llc_test.rail_mpc_->getStateMultibody()->zero() == llc_test.rail_mpc_->getStateRef()[i]);
  }
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1.0);

  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state == 1e-2);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_control == 1e-4);
}

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->loadParameters(MULTICOPTER_MPC_ROOT_DIR "/unittest/config/ocp/rail-mpc-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state == 4.5e-3);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(llc_test.rail_mpc_->getParams().w_control == 2.3e-4);
}

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  BOOST_CHECK(llc_test.rail_mpc_->getDifferentialRunningModels().size() == llc_test.rail_mpc_->getKnots() - 1);
  BOOST_CHECK(llc_test.rail_mpc_->getIntegratedRunningModels().size() == llc_test.rail_mpc_->getKnots() - 1);
  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
    // Check that the data differential model that std_vecto<integrated> is pointing to is the same that the data
    // member differential is pointing to
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            llc_test.rail_mpc_->getIntegratedRunningModels()[i]);
    BOOST_CHECK(int_model->get_differential() == llc_test.rail_mpc_->getDifferentialRunningModels()[i]);
    // Check the action model ptr of every node are pointing to different action model
    if (i < llc_test.rail_mpc_->getKnots() - 2) {
      BOOST_CHECK(llc_test.rail_mpc_->getDifferentialRunningModels()[i] !=
                  llc_test.rail_mpc_->getDifferentialRunningModels()[i + 1]);
      BOOST_CHECK(llc_test.rail_mpc_->getIntegratedRunningModels()[i] !=
                  llc_test.rail_mpc_->getIntegratedRunningModels()[i + 1]);
    } else {
      BOOST_CHECK(llc_test.rail_mpc_->getDifferentialRunningModels()[i] !=
                  llc_test.rail_mpc_->getDifferentialTerminalModel());
      BOOST_CHECK(llc_test.rail_mpc_->getIntegratedRunningModels()[i] !=
                  llc_test.rail_mpc_->getIntegratedTerminalModel());
    }
  }
  BOOST_CHECK(llc_test.rail_mpc_->getIntegratedTerminalModel()->get_differential() ==
              llc_test.rail_mpc_->getDifferentialTerminalModel());
}

BOOST_AUTO_TEST_CASE(create_costs_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
    BOOST_CHECK(llc_test.rail_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("state_error")
                    ->second->weight == llc_test.rail_mpc_->getParams().w_state);
    BOOST_CHECK(llc_test.rail_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("control_error")
                    ->second->weight == llc_test.rail_mpc_->getParams().w_control);
  }
  BOOST_CHECK(llc_test.rail_mpc_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("state_error")
                  ->second->weight == llc_test.rail_mpc_->getParams().w_state);
}

BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
        boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
            llc_test.rail_mpc_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("state_error")
                ->second->cost->get_activation());
    Eigen::VectorXd weights(llc_test.rail_mpc_->getParams().w_state_position.size() +
                            llc_test.rail_mpc_->getParams().w_state_orientation.size() +
                            llc_test.rail_mpc_->getParams().w_state_velocity_lin.size() +
                            llc_test.rail_mpc_->getParams().w_state_velocity_ang.size());
    weights << llc_test.rail_mpc_->getParams().w_state_position, llc_test.rail_mpc_->getParams().w_state_orientation,
        llc_test.rail_mpc_->getParams().w_state_velocity_lin, llc_test.rail_mpc_->getParams().w_state_velocity_ang;
    BOOST_CHECK(activation->get_weights() == weights);
  }
}

// Set reference before creating the problem

// XXX: As when creating the problem the reference trajecotry gets updated automatically from the trajectory generator
// the following test has no sense anymore. Leaved here in case we want to remove this automatic set in the future

// BOOST_AUTO_TEST_CASE(set_reference_test_1, *boost::unit_test::tolerance(1e-7)) {
//   RailMpcTest llc_test("mission-test.yaml");

//   std::vector<Eigen::VectorXd> state_reference_trajectory(llc_test.rail_mpc_->getKnots(),
//                                                           llc_test.rail_mpc_->getStateMultibody()->zero());
//   std::vector<Eigen::VectorXd> control_reference_trajectory(
//       llc_test.rail_mpc_->getKnots() - 1,
//       Eigen::VectorXd::Zero(llc_test.rail_mpc_->getActuation()->get_nu()));
//   for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
//     state_reference_trajectory[i](0) = i;
//     if (i < llc_test.rail_mpc_->getKnots() - 2) {
//       control_reference_trajectory[i](0) = i;
//     }
//   }

//   llc_test.rail_mpc_->setReferences(state_reference_trajectory, control_reference_trajectory);
//   llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

//   for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
//     boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
//         llc_test.rail_mpc_->getDifferentialRunningModels()[i]
//             ->get_costs()
//             ->get_costs()
//             .find("state_error")
//             ->second->cost);
//     BOOST_CHECK(state_reference_trajectory[i] == llc_test.rail_mpc_->getStateRef()[i]);
//     BOOST_CHECK(cost_state->get_reference<Eigen::VectorXd>() == llc_test.rail_mpc_->getStateRef()[i]);
//   }
//   boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
//       llc_test.rail_mpc_->getDifferentialTerminalModel()
//           ->get_costs()
//           ->get_costs()
//           .find("state_error")
//           ->second->cost);
//   BOOST_CHECK(state_reference_trajectory[llc_test.rail_mpc_->getKnots() - 1] ==
//               llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
//   BOOST_CHECK(cost_state->get_reference<Eigen::VectorXd>() ==
//               llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
// }

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(set_reference_test_2, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  std::vector<Eigen::VectorXd> state_reference_trajectory(llc_test.rail_mpc_->getKnots(),
                                                          llc_test.rail_mpc_->getStateMultibody()->zero());
  std::vector<Eigen::VectorXd> control_reference_trajectory(
      llc_test.rail_mpc_->getKnots() - 1, Eigen::VectorXd::Zero(llc_test.rail_mpc_->getActuation()->get_nu()));
  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
    state_reference_trajectory[i](0) = i;
    if (i < llc_test.rail_mpc_->getKnots() - 2) {
      control_reference_trajectory[i](0) = i;
    }
  }

  llc_test.rail_mpc_->setReferences(state_reference_trajectory, control_reference_trajectory);

  for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::CostModelState> cost_state =
        boost::static_pointer_cast<crocoddyl::CostModelState>(llc_test.rail_mpc_->getDifferentialRunningModels()[i]
                                                                  ->get_costs()
                                                                  ->get_costs()
                                                                  .find("state_error")
                                                                  ->second->cost);
    BOOST_CHECK(state_reference_trajectory[i] == llc_test.rail_mpc_->getStateRef()[i]);
    BOOST_CHECK(cost_state->get_reference<Eigen::VectorXd>() == llc_test.rail_mpc_->getStateRef()[i]);
  }
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      llc_test.rail_mpc_->getDifferentialTerminalModel()->get_costs()->get_costs().find("state_error")->second->cost);
  BOOST_CHECK(state_reference_trajectory[llc_test.rail_mpc_->getKnots() - 1] ==
              llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
  BOOST_CHECK(cost_state->get_reference<Eigen::VectorXd>() == llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
}

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(update_reference_trajectory_test, *boost::unit_test::tolerance(1e-7)) {
  // RailMpcTest llc_test("mission-test.yaml");

  // llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  // std::vector<Eigen::VectorXd> state_reference_trajectory(llc_test.rail_mpc_->getKnots(),
  //                                                         llc_test.rail_mpc_->getStateMultibody()->zero());
  // std::vector<Eigen::VectorXd> control_reference_trajectory(
  //     llc_test.rail_mpc_->getKnots() - 1,
  //     Eigen::VectorXd::Zero(llc_test.rail_mpc_->getActuation()->get_nu()));
  // for (std::size_t i = 0; i < llc_test.rail_mpc_->getKnots() - 1; ++i) {
  //   state_reference_trajectory[i](0) = i;
  //   if (i < llc_test.rail_mpc_->getKnots() - 2) {
  //     control_reference_trajectory[i](0) = i;
  //   }
  // }

  // llc_test.rail_mpc_->setReferences(state_reference_trajectory, control_reference_trajectory);
  // Eigen::VectorXd state = llc_test.rail_mpc_->getStateMultibody()->zero();
  // Eigen::VectorXd control = Eigen::VectorXd::Zero(llc_test.rail_mpc_->getActuation()->get_nu());
  // state(4) = 0.4563;
  // llc_test.rail_mpc_->updateReferences(state, control);

  // boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
  //     llc_test.rail_mpc_->getDifferentialTerminalModel()
  //         ->get_costs()
  //         ->get_costs()
  //         .find("state_error")
  //         ->second->cost);
  // BOOST_CHECK(state == llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
  // BOOST_CHECK(cost_state->get_reference<Eigen::VectorXd>() ==
  //             llc_test.rail_mpc_->getStateRef()[llc_test.rail_mpc_->getKnots() - 1]);
}

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(set_solver_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);
  BOOST_CHECK(llc_test.rail_mpc_->getSolver() != nullptr);
}

BOOST_AUTO_TEST_CASE(solve_test, *boost::unit_test::tolerance(1e-7)) {
  RailMpcTest llc_test("mission-test.yaml");

  llc_test.rail_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::Euler);

  Eigen::VectorXd state = llc_test.rail_mpc_->getStateMultibody()->zero();
  Eigen::VectorXd control = Eigen::VectorXd::Zero(llc_test.rail_mpc_->getActuation()->get_nu());
  state(2) = 0.3;
  control(2) = 0.2;
  std::vector<Eigen::VectorXd> state_reference(llc_test.rail_mpc_->getKnots(), state);
  std::vector<Eigen::VectorXd> control_reference(llc_test.rail_mpc_->getKnots() - 1, control);
  llc_test.rail_mpc_->setReferences(state_reference, control_reference);

  llc_test.rail_mpc_->setInitialState(state);
  BOOST_CHECK(llc_test.rail_mpc_->getInitialState() == state);

  llc_test.rail_mpc_->solve();
  // BOOST_CHECK(llc_test.rail_mpc_->getProblem()->get_x0() == state);
  // BOOST_CHECK(llc_test.rail_mpc_->getSolver()->get_stop() < 1e-7);
}

BOOST_AUTO_TEST_SUITE_END()