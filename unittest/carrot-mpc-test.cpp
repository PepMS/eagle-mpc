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
                   const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params,
                   const boost::shared_ptr<multicopter_mpc::Mission>& mission)
      : multicopter_mpc::CarrotMpc(model, mc_params, mission) {}

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
  const boost::shared_ptr<crocoddyl::ActionModelAbstract>& getIntegratedTerminalModel() { return int_model_terminal_; }
  const std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>::iterator& getIterator() {
    return diff_model_iter_;
  }

  const boost::shared_ptr<crocoddyl::SolverDDP>& getSolver() { return solver_; }

  const bool& getHasMotionRef() { return has_motion_ref_; }

  void initializeTrajectoryGen() { initializeTrajectoryGenerator(); }

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

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(multirotor_yaml_path);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

    mc_mission_ = boost::make_shared<multicopter_mpc::Mission>(mc_model_->nq + mc_model_->nv);
    mc_mission_->fillWaypoints(mission_yaml_path);

    dt_ = 1e-2;
    n_knots_ = 101;
    carrot_mpc_ = boost::make_shared<CarrotMpcDerived>(mc_model_, mc_params_, mc_mission_);
    carrot_mpc_->setTimeStep(dt_);
    carrot_mpc_->setNumberKnots(n_knots_);
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

  carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getMission() ==
              carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission());
}

BOOST_AUTO_TEST_CASE(initialize_weights_test, *boost::unit_test::tolerance(1e-7)) {
  // First waypoint after MPC Horizon: all false
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-1.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
    }
  }

  // First waypoint coinciding with the MPC Horizon: last true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-2.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      if (i == carrot_mpc_test.carrot_mpc_->getKnots() - 1) {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == true);
      } else {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
      }
    }
  }

  // MPC Horizon after middle waypoint: Wp true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-3.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
    carrot_mpc_test.carrot_mpc_->initializeTerminalW();

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      if (i == carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0] or
          i == carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1]) {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == true);
      } else {
        BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
      }
    }
  }

  // Last Waypoint before the MPC Horizon: Wp true and tail true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-4.yaml");

    carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
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

  carrot_mpc_test.carrot_mpc_->initializeTrajectoryGen();
  carrot_mpc_test.carrot_mpc_->initializeTerminalW();

  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->existsTerminalWeight() == true);
}

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  // First waypoint after MPC Horizon
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-1.yaml");

    carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                               multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);

    // Check that the differential running vector has ALL nodes (terminal node included)
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels().size() ==
                carrot_mpc_test.carrot_mpc_->getKnots());
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getIntegratedRunningModels().size() ==
                carrot_mpc_test.carrot_mpc_->getKnots());

    // Check that the differential running vector from the problem has only running nodes
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
      boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
          boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->cost);
      // Reference

      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .head(3)));
      BOOST_CHECK(
          cost_pose->get_reference<crocoddyl::FramePlacement>().placement.rotation() ==
          Eigen::Quaterniond(static_cast<Eigen::Vector4d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                              ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                              .segment(3, 7)))
              .toRotationMatrix());
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.linear() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .segment(7, 10)));
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .segment(10, 13)));
      // Weights
      bool active = i == carrot_mpc_test.carrot_mpc_->getKnots() - 1;

      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("pose_desired")
                      ->second->active == active);
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("vel_desired")
                      ->second->active == active);
    }
  }
  // First waypoint coinciding with the MPC Horizon: last true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-2.yaml");

    carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                               multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
          boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->cost);
      boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
          boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->cost);

      // Reference
      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .head(3)));
      BOOST_CHECK(
          cost_pose->get_reference<crocoddyl::FramePlacement>().placement.rotation() ==
          Eigen::Quaterniond(static_cast<Eigen::Vector4d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                              ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                              .segment(3, 7)))
              .toRotationMatrix());
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.linear() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .segment(7, 10)));
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular() ==
                  static_cast<Eigen::Vector3d>(carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()
                                                   ->getState(carrot_mpc_test.carrot_mpc_->getKnots() - 1)
                                                   .segment(10, 13)));
      // Weights
      bool active = i == carrot_mpc_test.carrot_mpc_->getKnots() - 1;

      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("pose_desired")
                      ->second->active == active);
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("vel_desired")
                      ->second->active == active);
    }
  }

  // MPC Horizon after middle waypoint: Wp true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-3.yaml");

    carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                               multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);
    ;

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
          boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->cost);
      boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
          boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->cost);
      // Reference
      Eigen::Vector3d pos_ref;
      Eigen::Quaterniond quat_ref;
      Eigen::Vector3d vel_lin_ref;
      Eigen::Vector3d vel_ang_ref;
      int knot_idx;
      if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0]) {
        knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0];
      } else if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1]) {
        knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1];
      } else {
        knot_idx = carrot_mpc_test.carrot_mpc_->getKnots() - 1;
      }
      pos_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).head(3));
      quat_ref = Eigen::Quaterniond(static_cast<Eigen::Vector4d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(3, 7)));
      vel_lin_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(7, 10));
      vel_ang_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(10, 13));
      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation() == pos_ref);
      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.rotation() ==
                  quat_ref.toRotationMatrix());
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.linear() == vel_lin_ref);
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular() == vel_ang_ref);
      // Weights
      bool active = (i == knot_idx || i == carrot_mpc_test.carrot_mpc_->getKnots() - 1);
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("pose_desired")
                      ->second->active == active);
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("vel_desired")
                      ->second->active == active);
    }
  }

  // Last Waypoint before the MPC Horizon: Wp true and tail true
  {
    CarrotMpcTest carrot_mpc_test("carrot-mpc-test-4.yaml");

    carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                               multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);
    ;

    for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getTerminalWeights().size(); ++i) {
      boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
          boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->cost);
      boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
          boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
              carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->cost);
      // Reference
      Eigen::Vector3d pos_ref;
      Eigen::Quaterniond quat_ref;
      Eigen::Vector3d vel_lin_ref;
      Eigen::Vector3d vel_ang_ref;
      int knot_idx;
      if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0]) {
        knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0];
      } else if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1]) {
        knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1];
      } else if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[2]) {
        knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[2];
      } else {
        knot_idx = i;
      }
      pos_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).head(3));
      quat_ref = Eigen::Quaterniond(static_cast<Eigen::Vector4d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(3, 7)));
      vel_lin_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(7, 10));
      vel_ang_ref = static_cast<Eigen::Vector3d>(
          carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(10, 13));
      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation() == pos_ref);
      BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.rotation() ==
                  quat_ref.toRotationMatrix());
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.linear() == vel_lin_ref);
      BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular() == vel_ang_ref);
      // Weights
      bool active = i == knot_idx;
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("pose_desired")
                      ->second->active == active);
      BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("vel_desired")
                      ->second->active == active);
    }
  }
}

BOOST_AUTO_TEST_CASE(update_problem_weight_test, *boost::unit_test::tolerance(1e-7)) {
  // MPC Horizon after middle waypoint: Wp true

  CarrotMpcTest carrot_mpc_test("carrot-mpc-test-3.yaml");

  carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                             multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);
  std::size_t trajectory_idx = carrot_mpc_test.carrot_mpc_->getKnots();
  std::vector<bool> initial_weights = carrot_mpc_test.carrot_mpc_->getTerminalWeights();
  carrot_mpc_test.carrot_mpc_->updateProblem(trajectory_idx);
  for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots() - 1; ++i) {
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == initial_weights[i + 1]);
  }
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights().back() == false);

  for (std::size_t i = 0; i < 129; ++i) {
    ++trajectory_idx;
    carrot_mpc_test.carrot_mpc_->updateProblem(trajectory_idx);
  }
  for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots() - 1; ++i) {
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights()[i] == false);
  }
  BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getTerminalWeights().back() == true);
}

BOOST_AUTO_TEST_CASE(update_problem_references_test, *boost::unit_test::tolerance(1e-7)) {
  // MPC Horizon after middle waypoint: Wp true

  CarrotMpcTest carrot_mpc_test("carrot-mpc-test-3.yaml");

  carrot_mpc_test.carrot_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP,
                                             multicopter_mpc::IntegratorTypes::Euler, carrot_mpc_test.dt_);
  std::size_t trajectory_idx = carrot_mpc_test.carrot_mpc_->getKnots();
  carrot_mpc_test.carrot_mpc_->updateProblem(trajectory_idx);

  for (std::size_t i = 0; i < carrot_mpc_test.carrot_mpc_->getKnots(); ++i) {
    boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
        boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("pose_desired")
                ->second->cost);
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("vel_desired")
                ->second->cost);
    // Reference
    Eigen::Vector3d pos_ref;
    Eigen::Quaterniond quat_ref;
    Eigen::Vector3d vel_lin_ref;
    Eigen::Vector3d vel_ang_ref;
    int knot_idx;
    if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0] - 1) {
      knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[0];
    } else if (i <= carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1] - 1) {
      knot_idx = carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getMission()->getWpTrajIdx()[1];
    } else {
      knot_idx = carrot_mpc_test.carrot_mpc_->getKnots();
    }
    pos_ref = static_cast<Eigen::Vector3d>(
        carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).head(3));
    quat_ref = Eigen::Quaterniond(static_cast<Eigen::Vector4d>(
        carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(3, 7)));
    vel_lin_ref = static_cast<Eigen::Vector3d>(
        carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(7, 10));
    vel_ang_ref = static_cast<Eigen::Vector3d>(
        carrot_mpc_test.carrot_mpc_->getTrajectoryGenerator()->getState(knot_idx).segment(10, 13));
    BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation() == pos_ref);
    BOOST_CHECK(cost_pose->get_reference<crocoddyl::FramePlacement>().placement.rotation() ==
                quat_ref.toRotationMatrix());
    BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.linear() == vel_lin_ref);
    BOOST_CHECK(cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular() == vel_ang_ref);
    // Weights
    bool active = (i == knot_idx - 1 || i == carrot_mpc_test.carrot_mpc_->getKnots() - 1);
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("pose_desired")
                    ->second->active == active);
    BOOST_CHECK(carrot_mpc_test.carrot_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("vel_desired")
                    ->second->active == active);
  }
}

BOOST_AUTO_TEST_SUITE_END()