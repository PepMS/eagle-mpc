#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/mpc/carrot-mpc.hpp"
#include "multicopter_mpc/mission.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class CarrotMpcDerived : public multicopter_mpc::CarrotMpc {
 public:
  CarrotMpcDerived(const boost::shared_ptr<pinocchio::Model>& model,
                   const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params, const double& dt,
                   const boost::shared_ptr<multicopter_mpc::Mission>& mission, std::size_t& n_knots)
      : multicopter_mpc::CarrotMpc(model, mc_params, dt, mission, n_knots) {}

  ~CarrotMpcDerived(){};

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
  const std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>::iterator& getIterator() {
    return diff_model_iter_;
  }

  const boost::shared_ptr<crocoddyl::SolverAbstract>& getSolver() { return solver_; }

  const bool& getHasMotionRef() { return has_motion_ref_; }

  void initializeTrajectoryGen(const multicopter_mpc::SolverTypes::Type& solver_type) {
    initializeTrajectoryGenerator(solver_type);
  }

  void initializeTerminalW() { initializeTerminalWeights(); }

  const std::vector<bool>& getTerminalWeights() { return terminal_weights_idx_; }
};

class CarrotMpcTest {
 public:
  CarrotMpcTest(const std::string& mission_name) {
    std::string multirotor_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/multirotor/iris.yaml";
    std::string mission_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/mission/" + mission_name;

    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                                pinocchio::JointModelFreeFlyer(), model_);

    yaml_parser::ParserYAML yaml_file(multirotor_yaml_path, "", true);
    yaml_parser::ParamsServer server_params(yaml_file.getParams());

    yaml_parser::ParserYAML mission_yaml_file(mission_yaml_path, "", true);
    yaml_parser::ParamsServer mission_server_params(mission_yaml_file.getParams());

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(server_params);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

    mc_mission_ = boost::make_shared<multicopter_mpc::Mission>(mc_model_->nq + mc_model_->nv);
    mc_mission_->fillWaypoints(mission_server_params);
    mc_mission_->fillInitialState(mission_server_params);

    dt_ = 1e-2;
    n_knots_ = 101;
    carrot_mpc_ = boost::make_shared<CarrotMpcDerived>(mc_model_, mc_params_, dt_, mc_mission_, n_knots_);
  }

  ~CarrotMpcTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;
  boost::shared_ptr<multicopter_mpc::Mission> mc_mission_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<CarrotMpcDerived> carrot_mpc_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  CarrotMpcTest carrot_mpc_test("mission-test.yaml");

  // Ocp_Base constructor
  BOOST_CHECK(carrot_mpc_test.mc_model_ == carrot_mpc_test.carrot_mpc_->getModel());
  BOOST_CHECK(carrot_mpc_test.mc_params_ == carrot_mpc_test.carrot_mpc_->getMcParams());
  BOOST_CHECK(carrot_mpc_test.dt_ == carrot_mpc_test.carrot_mpc_->getTimeStep());
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getStateMultibody() != nullptr);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(carrot_mpc_test.carrot_mpc_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(carrot_mpc_test.carrot_mpc_->getActuation()->get_nu());
  tau_lb.head(carrot_mpc_test.mc_params_->n_rotors_).fill(carrot_mpc_test.mc_params_->min_thrust_);
  tau_ub.head(carrot_mpc_test.mc_params_->n_rotors_).fill(carrot_mpc_test.mc_params_->max_thrust_);
  BOOST_CHECK(carrot_mpc_test.mc_params_->n_rotors_ == carrot_mpc_test.carrot_mpc_->getActuationLowerBounds().size());
  BOOST_CHECK(carrot_mpc_test.mc_params_->n_rotors_ == carrot_mpc_test.carrot_mpc_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == carrot_mpc_test.carrot_mpc_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == carrot_mpc_test.carrot_mpc_->getActuationUpperBounds());

  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getStateMultibody()->zero() ==
              carrot_mpc_test.carrot_mpc_->getInitialState());
  BOOST_CHECK(carrot_mpc_test.mc_model_->getFrameId(carrot_mpc_test.mc_params_->base_link_name_) ==
              carrot_mpc_test.carrot_mpc_->getBaseLinkId());

  // Low Level constructors
  BOOST_CHECK(carrot_mpc_test.n_knots_ == carrot_mpc_test.carrot_mpc_->getKnots());
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator() != nullptr);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission() == carrot_mpc_test.mc_mission_);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights().size() == carrot_mpc_test.carrot_mpc_->getKnots());
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  CarrotMpcTest carrot_mpc_test("mission-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1000.0);

  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_running == 1e-6);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_control_running == 1e-4);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_pos_running == 1e-2);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_vel_running == 1e-2);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal == 100);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal == 10);
}

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  CarrotMpcTest carrot_mpc_test("mission-test.yaml");

  carrot_mpc_test.carrot_mpc_->loadParameters(MULTICOPTER_MPC_ROOT_DIR "/unittest/config/ocp/carrot-mpc-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_state_running == 1.234);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_control_running == 2.345);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_pos_running == 3.456);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_vel_running == 4.567);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal == 5.678);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal == 6.789);
}

BOOST_AUTO_TEST_CASE(initialize_tg_test, *boost::unit_test::tolerance(1e-7)) {
  CarrotMpcTest carrot_mpc_test("takeoff.yaml");

  carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getMission() ==
              carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission());
}

BOOST_AUTO_TEST_CASE(initialize_weights_test, *boost::unit_test::tolerance(1e-7)) {
  // First waypoint after MPC Horizon: all false
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-1.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
    }
  }

  // First waypoint coinciding with the MPC Horizon: last true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-2.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      if (i == carrot_mpc_test.carrot_mpc_->getKnots() - 1) {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == true);
      } else {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
      }
    }
  }

  // Last Waypoint before the MPC Horizon: Wp true and tail true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-4.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      if (i == carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0] or
          i == carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1] or
          i >= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[2]) {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == true);
      } else {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(exists_terminal_weight_test, *boost::unit_test::tolerance(1e-7)) {
  CarrotMpcTest carrot_mpc_test("carrot-mpc-test-2.yaml");

  carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);
  carrot_mpc_test.carrot_mpc_->initializeTerminalW();

  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->existsTerminalWeight() == true);
}

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  // First waypoint after MPC Horizon: all false
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-1.yaml");

    carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

    // Check that the differentialrunning vector has ALL nodes (terminal node included)
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels().size() ==
                carrot_mpc_test.carrot_mpc_->getKnots());
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getIntegratedRunningModels().size() ==
                carrot_mpc_test.carrot_mpc_->getKnots());

    // Check that the differentialrunning vector has only running nodes (terminal node included)
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getProblem()->get_runningModels().size() ==
                carrot_mpc_test.carrot_mpc_->getKnots() - 1);

    // Check that the last of the running models and the terminal model are pointing at the same object
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels().back() ==
                carrot_mpc_test.carrot_mpc_->getDifferentialTerminalModel());

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
          boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->cost);
      BOOST_CHECK(cost_pose->get_Mref().oMf.translation() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(
                      carrot_mpc_test.carrot_mpc_->getKnots() - 1).head(3)));
      // TODO: Check the rest of the state
    }
    {
      // As done with the terminal weight vector, check the other mission test
    }
  }
}

// BOOST_AUTO_TEST_CASE(create_costs_weights_test, *boost::unit_test::tolerance(1e-7)) {
//   CarrotMpcTest carrot_mpc_test("takeoff.yaml");

//   carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots() - 1; ++i) {
//     BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
//                     ->get_costs()
//                     ->get_costs()
//                     .find("state_reg")
//                     ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_state_running);
//     BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
//                     ->get_costs()
//                     ->get_costs()
//                     .find("control_reg")
//                     ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_control_running);
//     BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
//                     ->get_costs()
//                     ->get_costs()
//                     .find("pose_desired")
//                     ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_running);
//     BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
//                     ->get_costs()
//                     ->get_costs()
//                     .find("vel_desired")
//                     ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_running);
//   }
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialTerminalModel()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("pose_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialTerminalModel()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("vel_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal);
// }

// This checks that the weights for the weighted quadratic barrier in the state costs are properly set
// BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
//   CarrotMpcTest carrot_mpc_test("takeoff.yaml");

//   carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots() - 1; ++i) {
//     boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
//         boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
//             carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
//                 ->get_costs()
//                 ->get_costs()
//                 .find("state_reg")
//                 ->second->cost->get_activation());
//     Eigen::VectorXd weights(carrot_mpc_test.carrot_mpc_->getParams().w_state_position.size() +
//                             carrot_mpc_test.carrot_mpc_->getParams().w_state_orientation.size() +
//                             carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_lin.size() +
//                             carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_ang.size());
//     weights << carrot_mpc_test.carrot_mpc_->getParams().w_state_position,
//     carrot_mpc_test.carrot_mpc_->getParams().w_state_orientation,
//         carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_lin,
//         carrot_mpc_test.carrot_mpc_->getParams().w_state_velocity_ang;
//     BOOST_CHECK(activation->get_weights() == weights);
//   }
// }

// // This test is to check that the weights are properly updated when the method updateWeights() is called
// BOOST_AUTO_TEST_CASE(update_weights_test, *boost::unit_test::tolerance(1e-7)) {
//   CarrotMpcTest carrot_mpc_test("takeoff.yaml");

//   carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   // Before calling the updateProblem() last node should have the terminal weight
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()
//                   .back()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("pose_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()
//                   .back()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("vel_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[carrot_mpc_test.carrot_mpc_->getKnots() -
//   2]
//                   ->get_costs()
//                   ->get_costs()
//                   .find("pose_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_running);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[carrot_mpc_test.carrot_mpc_->getKnots() -
//   2]
//                   ->get_costs()
//                   ->get_costs()
//                   .find("vel_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_running);

//   carrot_mpc_test.carrot_mpc_->updateProblem(carrot_mpc_test.carrot_mpc_->getKnots());

//   // After calling, last node running weights and the penultimate terminal weights
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()
//                   .back()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("pose_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_running);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()
//                   .back()
//                   ->get_costs()
//                   ->get_costs()
//                   .find("vel_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_running);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[carrot_mpc_test.carrot_mpc_->getKnots() -
//   2]
//                   ->get_costs()
//                   ->get_costs()
//                   .find("pose_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[carrot_mpc_test.carrot_mpc_->getKnots() -
//   2]
//                   ->get_costs()
//                   ->get_costs()
//                   .find("vel_desired")
//                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal);
// }

// // This test is to check that the iterator indicating the terminal weight is the last node after calling
// // updateProblem() n_knots - 1 times
// BOOST_AUTO_TEST_CASE(update_iterator_cyclic_test, *boost::unit_test::tolerance(1e-7)) {
//   CarrotMpcTest carrot_mpc_test("takeoff.yaml");

//   carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   for (int i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots() - 1; ++i) {
//     carrot_mpc_test.carrot_mpc_->updateProblem(carrot_mpc_test.carrot_mpc_->getKnots());
//   }

//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getIterator() ==
//   carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels().end() - 1);
// }

// BOOST_AUTO_TEST_CASE(update_terminal_cost_test, *boost::unit_test::tolerance(1e-7)) {
//   CarrotMpcTest carrot_mpc_test("takeoff.yaml");

//   carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   std::size_t trajectory_cursor = carrot_mpc_test.carrot_mpc_->getKnots();

//   carrot_mpc_test.carrot_mpc_->updateProblem(trajectory_cursor);

//   boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
//       boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
//           carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()
//               .back()
//               ->get_costs()
//               ->get_costs()
//               .find("pose_desired")
//               ->second->cost);
//   BOOST_CHECK(cost_pose->get_Mref().oMf == carrot_mpc_test.carrot_mpc_->getMission()->getWaypoints()[1].pose);
//   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getHasMotionRef() == true);
// }

// The hovering case should be implemented in the updateProblem case
BOOST_AUTO_TEST_CASE(hovering_state_test, *boost::unit_test::tolerance(1e-7)) {
  // CarrotMpcTest carrot_mpc_test("takeoff.yaml");

  // carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  // std::size_t trajectory_cursor = carrot_mpc_test.carrot_mpc_->getKnots() - 1;

  // for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getTotalKnots() -
  //                                 carrot_mpc_test.carrot_mpc_->getKnots() + 1;
  //      ++i) {
  //   ++trajectory_cursor;
  //   carrot_mpc_test.carrot_mpc_->updateProblem(trajectory_cursor);
  // }

  // for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels().size(); ++i) {
  //   boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
  //       boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
  //           carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
  //               ->get_costs()
  //               ->get_costs()
  //               .find("pose_desired")
  //               ->second->cost);
  //   boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
  //       boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
  //           carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
  //               ->get_costs()
  //               ->get_costs()
  //               .find("vel_desired")
  //               ->second->cost);

  //   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
  //                   ->get_costs()
  //                   ->get_costs()
  //                   .find("pose_desired")
  //                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_pos_terminal);
  //   BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
  //                   ->get_costs()
  //                   ->get_costs()
  //                   .find("vel_desired")
  //                   ->second->weight == carrot_mpc_test.carrot_mpc_->getParams().w_vel_terminal);
  //   BOOST_CHECK(cost_pose->get_Mref().oMf == carrot_mpc_test.carrot_mpc_->getMission()->getWaypoints().back().pose);
  //   BOOST_CHECK(cost_vel->get_vref().oMf == carrot_mpc_test.carrot_mpc_->getMission()->getWaypoints().back().vel);
  // }
}

BOOST_AUTO_TEST_SUITE_END()