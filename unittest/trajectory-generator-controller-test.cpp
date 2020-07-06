#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/trajectory-generator-controller.hpp"
#include "multicopter_mpc/mission.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class TGControllerDerived : public multicopter_mpc::TrajectoryGeneratorController {
 public:
  TGControllerDerived(const boost::shared_ptr<pinocchio::Model>& model,
                      const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params, const double& dt,
                      const boost::shared_ptr<multicopter_mpc::Mission>& mission, std::size_t& n_knots)
      : multicopter_mpc::TrajectoryGeneratorController(model, mc_params, dt, mission, n_knots) {}

  ~TGControllerDerived(){};

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
};

class TGControllerTest {
 public:
  TGControllerTest(const std::string& mission_name) {
    std::string multirotor_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/iris.yaml";
    std::string mission_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/" + mission_name;

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
    n_knots_ = 100;
    tg_controller_ = boost::make_shared<TGControllerDerived>(mc_model_, mc_params_, dt_, mc_mission_, n_knots_);
  }

  ~TGControllerTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;
  boost::shared_ptr<multicopter_mpc::Mission> mc_mission_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<TGControllerDerived> tg_controller_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  // Ocp_Base constructor
  BOOST_CHECK(tgc_test.mc_model_ == tgc_test.tg_controller_->getModel());
  BOOST_CHECK(tgc_test.mc_params_ == tgc_test.tg_controller_->getMcParams());
  BOOST_CHECK(tgc_test.dt_ == tgc_test.tg_controller_->getTimeStep());
  BOOST_CHECK(tgc_test.tg_controller_->getStateMultibody() != nullptr);
  BOOST_CHECK(tgc_test.tg_controller_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(tgc_test.tg_controller_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(tgc_test.tg_controller_->getActuation()->get_nu());
  tau_lb.head(tgc_test.mc_params_->n_rotors_).fill(tgc_test.mc_params_->min_thrust_);
  tau_ub.head(tgc_test.mc_params_->n_rotors_).fill(tgc_test.mc_params_->max_thrust_);
  BOOST_CHECK(tgc_test.mc_params_->n_rotors_ == tgc_test.tg_controller_->getActuationLowerBounds().size());
  BOOST_CHECK(tgc_test.mc_params_->n_rotors_ == tgc_test.tg_controller_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == tgc_test.tg_controller_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == tgc_test.tg_controller_->getActuationUpperBounds());

  BOOST_CHECK(tgc_test.tg_controller_->getStateMultibody()->zero() == tgc_test.tg_controller_->getInitialState());
  BOOST_CHECK(tgc_test.mc_model_->getFrameId(tgc_test.mc_params_->base_link_name_) ==
              tgc_test.tg_controller_->getBaseLinkId());

  // Low Level constructor
  BOOST_CHECK(tgc_test.tg_controller_->getMission() != nullptr);
  BOOST_CHECK(tgc_test.n_knots_ == tgc_test.tg_controller_->getKnots());
  BOOST_CHECK(tgc_test.tg_controller_->getPoseRef().frame == tgc_test.tg_controller_->getBaseLinkId());
  BOOST_CHECK(tgc_test.tg_controller_->getPoseRef().oMf == tgc_test.mc_mission_->waypoints_[0].pose);
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1000.0);

  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_position == w_position);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_running == 1e-6);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_control_running == 1e-4);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_pos_running == 1e-2);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_vel_running == 1e-2);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_pos_terminal == 100);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_vel_terminal == 10);
}

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  std::string params_yaml_path = MULTICOPTER_MPC_ROOT_DIR "/unittest/config/trajectory-generator-test.yaml";
  yaml_parser::ParserYAML yaml_file(params_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  tgc_test.tg_controller_->loadParameters(server_params);

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_position == w_position);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_state_running == 1.234);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_control_running == 2.345);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_pos_running == 3.456);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_vel_running == 4.567);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_pos_terminal == 5.678);
  BOOST_CHECK(tgc_test.tg_controller_->getParams().w_vel_terminal == 6.789);
}

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels().size() == tgc_test.tg_controller_->getKnots());
  BOOST_CHECK(tgc_test.tg_controller_->getIntegratedRunningModels().size() == tgc_test.tg_controller_->getKnots());

  // Differential and Integrated->Differential are pointing at the same place
  for (std::size_t i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    // Check that the data differential model that std_vector<integrated> is pointing to is the same that the data
    // member differential is pointing to
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            tgc_test.tg_controller_->getIntegratedRunningModels()[i]);
    BOOST_CHECK(int_model->get_differential() == tgc_test.tg_controller_->getDifferentialRunningModels()[i]);
    // Check the action model ptr of every node are pointing to different action model
    if (i < tgc_test.tg_controller_->getKnots() - 2) {
      BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i] !=
                  tgc_test.tg_controller_->getDifferentialRunningModels()[i + 1]);
      BOOST_CHECK(tgc_test.tg_controller_->getIntegratedRunningModels()[i] !=
                  tgc_test.tg_controller_->getIntegratedRunningModels()[i + 1]);
    } else {
      BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i] !=
                  tgc_test.tg_controller_->getDifferentialTerminalModel());
      BOOST_CHECK(tgc_test.tg_controller_->getIntegratedRunningModels()[i] !=
                  tgc_test.tg_controller_->getIntegratedTerminalModel());
    }
  }
  BOOST_CHECK(tgc_test.tg_controller_->getIntegratedTerminalModel()->get_differential() ==
              tgc_test.tg_controller_->getDifferentialTerminalModel());

  // Last of running and terminal are pointing at the same place
  BOOST_CHECK(tgc_test.tg_controller_->getIntegratedTerminalModel() ==
              tgc_test.tg_controller_->getIntegratedRunningModels().back());
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialTerminalModel() ==
              tgc_test.tg_controller_->getDifferentialRunningModels().back());
}

BOOST_AUTO_TEST_CASE(create_costs_weights_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("state_reg")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_state_running);
    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("control_reg")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_control_running);
    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("pose_desired")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_pos_running);
    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("vel_desired")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_vel_running);
  }
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_pos_terminal);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_vel_terminal);
}

// This test checks whether the reference in the placement cost (from problem) are the same as in the class. This
// should be probably be removed since it might phappen that all references are not the same for all the nodes
BOOST_AUTO_TEST_CASE(create_costs_reference_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
        boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("pose_desired")
                ->second->cost);
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("vel_desired")
                ->second->cost);

    BOOST_CHECK(cost_pose->get_Mref().oMf == tgc_test.tg_controller_->getPoseRef().oMf);
    BOOST_CHECK(cost_pose->get_Mref().frame == tgc_test.tg_controller_->getPoseRef().frame);
    BOOST_CHECK(cost_vel->get_vref().oMf == tgc_test.tg_controller_->getVelocityRef().oMf);
    BOOST_CHECK(cost_vel->get_vref().frame == tgc_test.tg_controller_->getVelocityRef().frame);
  }
  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          tgc_test.tg_controller_->getDifferentialTerminalModel()
              ->get_costs()
              ->get_costs()
              .find("pose_desired")
              ->second->cost);
  boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
      boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
          tgc_test.tg_controller_->getDifferentialTerminalModel()
              ->get_costs()
              ->get_costs()
              .find("vel_desired")
              ->second->cost);

  BOOST_CHECK(cost_pose->get_Mref().oMf == tgc_test.tg_controller_->getPoseRef().oMf);
  BOOST_CHECK(cost_pose->get_Mref().frame == tgc_test.tg_controller_->getPoseRef().frame);
  BOOST_CHECK(cost_vel->get_vref().oMf == tgc_test.tg_controller_->getVelocityRef().oMf);
  BOOST_CHECK(cost_vel->get_vref().frame == tgc_test.tg_controller_->getVelocityRef().frame);
}

// This checks that the weights for the weighted quadratic barrier in the state costs are probperly set
BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
        boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
            tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("state_reg")
                ->second->cost->get_activation());
    Eigen::VectorXd weights(tgc_test.tg_controller_->getParams().w_state_position.size() +
                            tgc_test.tg_controller_->getParams().w_state_orientation.size() +
                            tgc_test.tg_controller_->getParams().w_state_velocity_lin.size() +
                            tgc_test.tg_controller_->getParams().w_state_velocity_ang.size());
    weights << tgc_test.tg_controller_->getParams().w_state_position,
        tgc_test.tg_controller_->getParams().w_state_orientation,
        tgc_test.tg_controller_->getParams().w_state_velocity_lin,
        tgc_test.tg_controller_->getParams().w_state_velocity_ang;
    BOOST_CHECK(activation->get_weights() == weights);
  }
}

// This test is to check that the weights are properly updated when the method updateWeights() is called
BOOST_AUTO_TEST_CASE(update_weights_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  // Before calling the updateProblem() last node should have the terminal weight
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_pos_terminal);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_vel_terminal);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[tgc_test.tg_controller_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_pos_running);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[tgc_test.tg_controller_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_vel_running);

  tgc_test.tg_controller_->updateProblem(tgc_test.tg_controller_->getKnots());

  // After calling, last node running weights and the penultimate terminal weights
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_pos_running);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_vel_running);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[tgc_test.tg_controller_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_pos_terminal);
  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[tgc_test.tg_controller_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == tgc_test.tg_controller_->getParams().w_vel_terminal);
}

BOOST_AUTO_TEST_CASE(update_weights_steady_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::size_t trajectory_cursor = tgc_test.tg_controller_->getKnots() - 1;

  for (int i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    tgc_test.tg_controller_->updateProblem(tgc_test.tg_controller_->getKnots());
  }

  BOOST_CHECK(tgc_test.tg_controller_->getIterator() ==
              tgc_test.tg_controller_->getDifferentialRunningModels().end() - 1);
}

// This test is to check that the iterator indicating the terminal weight is the last node after calling
// updateProblem() n_knots - 1 times
BOOST_AUTO_TEST_CASE(update_weights_iterator_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (int i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    tgc_test.tg_controller_->updateProblem(tgc_test.tg_controller_->getKnots());
  }

  BOOST_CHECK(tgc_test.tg_controller_->getIterator() ==
              tgc_test.tg_controller_->getDifferentialRunningModels().end() - 1);
}

BOOST_AUTO_TEST_CASE(update_terminal_cost_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::size_t trajectory_cursor = tgc_test.tg_controller_->getKnots();

  tgc_test.tg_controller_->updateProblem(trajectory_cursor);

  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          tgc_test.tg_controller_->getDifferentialRunningModels()
              .back()
              ->get_costs()
              ->get_costs()
              .find("pose_desired")
              ->second->cost);
  BOOST_CHECK(cost_pose->get_Mref().oMf == tgc_test.tg_controller_->getMission()->waypoints_[1].pose);
  BOOST_CHECK(tgc_test.tg_controller_->getHasMotionRef() == false);

  trajectory_cursor += tgc_test.tg_controller_->getMission()->waypoints_[1].knots - 1;
  tgc_test.tg_controller_->updateProblem(trajectory_cursor);
  cost_pose = boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
      tgc_test.tg_controller_->getDifferentialRunningModels()
          .back()
          ->get_costs()
          ->get_costs()
          .find("pose_desired")
          ->second->cost);
  BOOST_CHECK(cost_pose->get_Mref().oMf == tgc_test.tg_controller_->getMission()->waypoints_[2].pose);
  BOOST_CHECK(tgc_test.tg_controller_->getHasMotionRef() == false);
}

BOOST_AUTO_TEST_CASE(hovering_state_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test("mission-1wp-test.yaml");

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::size_t trajectory_cursor = tgc_test.tg_controller_->getKnots() - 1;

  for (std::size_t i = 0; i < tgc_test.tg_controller_->getKnots() - 1; ++i) {
    ++trajectory_cursor;
    tgc_test.tg_controller_->updateProblem(trajectory_cursor);
  }

  for (std::size_t i = 0; i < tgc_test.tg_controller_->getDifferentialRunningModels().size(); ++i) {
    boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
        boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("pose_desired")
                ->second->cost);
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("vel_desired")
                ->second->cost);

    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("pose_desired")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_pos_terminal);
    BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("vel_desired")
                    ->second->weight == tgc_test.tg_controller_->getParams().w_vel_terminal);
    BOOST_CHECK(cost_pose->get_Mref().oMf == tgc_test.tg_controller_->getMission()->waypoints_.back().pose);
    BOOST_CHECK(cost_vel->get_vref().oMf == tgc_test.tg_controller_->getMission()->waypoints_.back().vel);
  }
}

BOOST_AUTO_TEST_SUITE_END()