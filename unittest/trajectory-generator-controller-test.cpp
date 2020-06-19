#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/trajectory-generator-controller.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class TGControllerDerived : public multicopter_mpc::TrajectoryGeneratorController {
 public:
  TGControllerDerived(const boost::shared_ptr<pinocchio::Model>& model,
                      const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params, const double& dt,
                      std::size_t& n_knots)
      : multicopter_mpc::TrajectoryGeneratorController(model, mc_params, dt, n_knots) {}

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

  const boost::shared_ptr<crocoddyl::SolverAbstract>& getSolver() { return solver_; }
};

class TGControllerTest {
 public:
  TGControllerTest() {
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
    tg_controller_ = boost::make_shared<TGControllerDerived>(mc_model_, mc_params_, dt_, n_knots_);
  }

  ~TGControllerTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<TGControllerDerived> tg_controller_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

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
  BOOST_CHECK(tgc_test.n_knots_ == tgc_test.tg_controller_->getKnots());
  BOOST_CHECK(tgc_test.tg_controller_->getPoseRef().frame == tgc_test.tg_controller_->getBaseLinkId());
  BOOST_CHECK(tgc_test.tg_controller_->getPoseRef().oMf == pinocchio::SE3::Identity());
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

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
  TGControllerTest tgc_test;

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
  TGControllerTest tgc_test;

  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  BOOST_CHECK(tgc_test.tg_controller_->getDifferentialRunningModels().size() ==
              tgc_test.tg_controller_->getKnots() - 1);
  BOOST_CHECK(tgc_test.tg_controller_->getIntegratedRunningModels().size() == tgc_test.tg_controller_->getKnots() - 1);
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
}

BOOST_AUTO_TEST_CASE(create_costs_weights_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

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

BOOST_AUTO_TEST_CASE(create_costs_reference_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

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

BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

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

// Set reference before creating the problem
BOOST_AUTO_TEST_CASE(set_reference_test_1, *boost::unit_test::tolerance(1e-7)) {
  TGControllerTest tgc_test;

  Eigen::Vector3d pos_ref;
  pos_ref << 1, 2, 3;
  Eigen::Vector3d vel_lin_ref;
  vel_lin_ref << 4, 5, 6;
  Eigen::Vector3d vel_ang_ref;
  vel_ang_ref << 7, 8, 9;
  Eigen::Quaterniond quat_ref(0, 0, 1, 0);

  Eigen::VectorXd state_new(13);
  state_new << pos_ref, quat_ref.vec(), quat_ref.w(), vel_lin_ref, vel_ang_ref;
  
  pinocchio::SE3 pose_ref(quat_ref, pos_ref);
  pinocchio::Motion motion_ref(state_new.segment(7, 3), state_new.segment(10, 3));
  tgc_test.tg_controller_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);
  tgc_test.tg_controller_->updateReferences(state_new);
  
  BOOST_CHECK(pose_ref == tgc_test.tg_controller_->getPoseRef().oMf);
  BOOST_CHECK(motion_ref == tgc_test.tg_controller_->getVelocityRef().oMf);

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

// Set reference after creating the problem
BOOST_AUTO_TEST_CASE(set_reference_test_2, *boost::unit_test::tolerance(1e-7)) {}

// BOOST_AUTO_TEST_CASE(update_reference_trajectory_test, *boost::unit_test::tolerance(1e-7)) {

// }

// BOOST_AUTO_TEST_CASE(set_solver_test, *boost::unit_test::tolerance(1e-7)) {

// }

// BOOST_AUTO_TEST_CASE(solve_test, *boost::unit_test::tolerance(1e-7)) {

// }

BOOST_AUTO_TEST_SUITE_END()