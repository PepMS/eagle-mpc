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

    dt_ = 1e-2;
    mission_ = boost::make_shared<multicopter_mpc::Mission>(model_.nq + model_.nv);
    mission_->fillWaypoints(server_mission, dt_);
    mission_->fillInitialState(server_mission);

    mc_model_ = boost::make_shared<pinocchio::Model>(model_);

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
  BOOST_CHECK(tg_test.trajectory_generator_->getStateMultibody() != nullptr);
  BOOST_CHECK(tg_test.trajectory_generator_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(tg_test.trajectory_generator_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(tg_test.trajectory_generator_->getActuation()->get_nu());
  tau_lb.head(tg_test.mc_params_->n_rotors_).fill(tg_test.mc_params_->min_thrust_);
  tau_ub.head(tg_test.mc_params_->n_rotors_).fill(tg_test.mc_params_->max_thrust_);
  BOOST_CHECK(tg_test.mc_params_->n_rotors_ == tg_test.trajectory_generator_->getActuationLowerBounds().size());
  BOOST_CHECK(tg_test.mc_params_->n_rotors_ == tg_test.trajectory_generator_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == tg_test.trajectory_generator_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == tg_test.trajectory_generator_->getActuationUpperBounds());
  BOOST_CHECK(tg_test.trajectory_generator_->getStateMultibody()->zero() ==
              tg_test.trajectory_generator_->getInitialState());

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
  
  tg_test.trajectory_generator_->loadParameters( MULTICOPTER_MPC_ROOT_DIR "/unittest/config/trajectory-generator-test.yaml");

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

BOOST_AUTO_TEST_CASE(trajectory_create_differential_model_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::size_t knot_cursor = 0;
  for (std::vector<multicopter_mpc::WayPoint>::const_iterator wp =
           tg_test.trajectory_generator_->getMission()->getWaypoints().begin();
       wp != tg_test.trajectory_generator_->getMission()->getWaypoints().end(); ++wp) {
    for (std::size_t i = knot_cursor; i < knot_cursor + wp->knots - 2; ++i) {
      BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("state_reg")
                      ->second->weight == tg_test.trajectory_generator_->getParams().w_state_running);
      BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("control_reg")
                      ->second->weight == tg_test.trajectory_generator_->getParams().w_control_running);
      BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[i]
                      ->get_costs()
                      ->get_costs()
                      .find("pos_desired")
                      ->second->weight == tg_test.trajectory_generator_->getParams().w_pos_running);
      if (wp->vel != boost::none) {
        BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[i]
                        ->get_costs()
                        ->get_costs()
                        .find("vel_desired")
                        ->second->weight == tg_test.trajectory_generator_->getParams().w_vel_running);
      }
    }
    if (std::next(wp) != tg_test.trajectory_generator_->getMission()->getWaypoints().end()) {
      BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[knot_cursor + wp->knots - 1]
                      ->get_costs()
                      ->get_costs()
                      .find("pos_desired")
                      ->second->weight == tg_test.trajectory_generator_->getParams().w_pos_terminal);
      if (wp->vel != boost::none) {
        BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[knot_cursor + wp->knots - 1]
                        ->get_costs()
                        ->get_costs()
                        .find("vel_desired")
                        ->second->weight == tg_test.trajectory_generator_->getParams().w_vel_terminal);
      }
    } else {
      BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialTerminalModel()
                      ->get_costs()
                      ->get_costs()
                      .find("pos_desired")
                      ->second->weight == tg_test.trajectory_generator_->getParams().w_pos_terminal);
      if (wp->vel != boost::none) {
        BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialTerminalModel()
                        ->get_costs()
                        ->get_costs()
                        .find("vel_desired")
                        ->second->weight == tg_test.trajectory_generator_->getParams().w_vel_terminal);
      }
    }
    knot_cursor += wp->knots;
  }
}

BOOST_AUTO_TEST_CASE(trajectory_create_differential_model_state_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
      boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
          tg_test.trajectory_generator_->getDifferentialRunningModels()[0]
              ->get_costs()
              ->get_costs()
              .find("state_reg")
              ->second->cost->get_activation());

  Eigen::VectorXd weights(tg_test.trajectory_generator_->getParams().w_state_position.size() +
                          tg_test.trajectory_generator_->getParams().w_state_orientation.size() +
                          tg_test.trajectory_generator_->getParams().w_state_velocity_lin.size() +
                          tg_test.trajectory_generator_->getParams().w_state_velocity_ang.size());

  weights << tg_test.trajectory_generator_->getParams().w_state_position,
      tg_test.trajectory_generator_->getParams().w_state_orientation,
      tg_test.trajectory_generator_->getParams().w_state_velocity_lin,
      tg_test.trajectory_generator_->getParams().w_state_velocity_ang;

  BOOST_CHECK(activation->get_weights() == weights);
}

BOOST_AUTO_TEST_CASE(trajectory_create_problem_action_models_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels().size() ==
              tg_test.trajectory_generator_->getKnots() - 1);
  BOOST_CHECK(tg_test.trajectory_generator_->getIntegratedRunningModels().size() ==
              tg_test.trajectory_generator_->getKnots() - 1);

  // Check that the data differential model that std_vector<integrated> is pointing to is the same that the data
  // member differential is pointing to. Only with the 0 element as aftewrwards we will check that all in pointers in
  // the vector are pointing at the same place

  // Be aware that we could have several Waypoints, meaning different stages of the whole trajectory. Each
  // stage has its own models
  std::size_t knot_cursor = 0;
  for (std::vector<multicopter_mpc::WayPoint>::const_iterator wp =
           tg_test.trajectory_generator_->getMission()->getWaypoints().begin();
       wp != tg_test.trajectory_generator_->getMission()->getWaypoints().end(); ++wp) {
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_0 =
        boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            tg_test.trajectory_generator_->getIntegratedRunningModels()[knot_cursor]);
    BOOST_CHECK(int_model_0->get_differential() ==
                tg_test.trajectory_generator_->getDifferentialRunningModels()[knot_cursor]);
    if (knot_cursor > 0) {
      BOOST_CHECK(int_model_0 != tg_test.trajectory_generator_->getIntegratedRunningModels()[knot_cursor - 1]);
    }
    for (std::size_t i = knot_cursor; i < knot_cursor + wp->knots - 1; ++i) {
      // Check the action model ptr of every node are pointing to different action model
      if (i < tg_test.trajectory_generator_->getKnots() - 1) {
        BOOST_CHECK(tg_test.trajectory_generator_->getDifferentialRunningModels()[i] ==
                    int_model_0->get_differential());
        BOOST_CHECK(tg_test.trajectory_generator_->getIntegratedRunningModels()[i] == int_model_0);
      }
    }
    knot_cursor = knot_cursor == 0 ? knot_cursor + wp->knots : knot_cursor + wp->knots - 1;
  }
  BOOST_CHECK(tg_test.trajectory_generator_->getIntegratedTerminalModel()->get_differential() ==
              tg_test.trajectory_generator_->getDifferentialTerminalModel());
  BOOST_CHECK(tg_test.trajectory_generator_->getMission()->getInitialState() == tg_test.trajectory_generator_->getInitialState());
}

BOOST_AUTO_TEST_CASE(set_solver_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  BOOST_CHECK(tg_test.trajectory_generator_->getSolver() != nullptr);
}

BOOST_AUTO_TEST_CASE(solve_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  tg_test.trajectory_generator_->setSolverCallbacks(true);
  tg_test.trajectory_generator_->solve();
  BOOST_CHECK(tg_test.trajectory_generator_->getSolver()->get_xs().size() ==
              tg_test.trajectory_generator_->getKnots());
  BOOST_CHECK(tg_test.trajectory_generator_->getProblem()->get_x0() ==
              tg_test.trajectory_generator_->getInitialState());
  BOOST_CHECK(tg_test.trajectory_generator_->getSolver()->get_stop() < 1e-4);
}

BOOST_AUTO_TEST_CASE(hover_test, *boost::unit_test::tolerance(1e-7)) {
  TrajectoryGeneratorTest tg_test;

  tg_test.trajectory_generator_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  tg_test.trajectory_generator_->setSolverCallbacks(true);
  tg_test.trajectory_generator_->solve();

  Eigen::VectorXd hover_state = tg_test.trajectory_generator_->getState(tg_test.trajectory_generator_->getKnots() + 2);
  BOOST_CHECK(hover_state.head(3) == tg_test.trajectory_generator_->getSolver()->get_xs().back().head(3));
  BOOST_CHECK(hover_state.segment(3, 2) == Eigen::Vector2d::Zero());
  Eigen::Quaterniond quat(tg_test.trajectory_generator_->getSolver()->get_xs().back()(6), 0, 0,
                          tg_test.trajectory_generator_->getSolver()->get_xs().back()(5));
  quat.normalize();
  BOOST_CHECK(hover_state(5) == quat.z());
  BOOST_CHECK(hover_state(6) == quat.w());
  BOOST_CHECK(hover_state.segment(7, 6) == Eigen::VectorXd::Zero(6));
}

BOOST_AUTO_TEST_SUITE_END()