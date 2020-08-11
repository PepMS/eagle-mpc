#define BOOST_TEST_MODULE trajectory_generator_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/mpc/picewise-mpc.hpp"
#include "multicopter_mpc/mission.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_trajectory_generator_test)

class PiceWiseMpcDerived : public multicopter_mpc::PiceWiseMpc {
 public:
  PiceWiseMpcDerived(const boost::shared_ptr<pinocchio::Model>& model,
                     const boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams>& mc_params, const double& dt,
                     const boost::shared_ptr<multicopter_mpc::Mission>& mission, std::size_t& n_knots)
      : multicopter_mpc::PiceWiseMpc(model, mc_params, dt, mission, n_knots) {}

  ~PiceWiseMpcDerived(){};

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

  const boost::shared_ptr<crocoddyl::SolverDDP>& getSolver() { return solver_; }

  const bool& getHasMotionRef() { return has_motion_ref_; }

  void initializeTrajectoryGen(const multicopter_mpc::SolverTypes::Type& solver_type) {
    initializeTrajectoryGenerator(solver_type);
  }
};

class PiceWiseMpcTest {
 public:
  PiceWiseMpcTest(const std::string& mission_name) {
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
    pw_mpc_ = boost::make_shared<PiceWiseMpcDerived>(mc_model_, mc_params_, dt_, mc_mission_, n_knots_);
  }

  ~PiceWiseMpcTest() {}

  pinocchio::Model model_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;
  boost::shared_ptr<multicopter_mpc::Mission> mc_mission_;

  double dt_;
  std::size_t n_knots_;

  boost::shared_ptr<PiceWiseMpcDerived> pw_mpc_;
};

BOOST_AUTO_TEST_CASE(constructor_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("mission-test.yaml");

  // Ocp_Base constructor
  BOOST_CHECK(pw_mpc_test.mc_model_ == pw_mpc_test.pw_mpc_->getModel());
  BOOST_CHECK(pw_mpc_test.mc_params_ == pw_mpc_test.pw_mpc_->getMcParams());
  BOOST_CHECK(pw_mpc_test.dt_ == pw_mpc_test.pw_mpc_->getTimeStep());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getStateMultibody() != nullptr);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getActuation() != nullptr);

  Eigen::VectorXd tau_lb(pw_mpc_test.pw_mpc_->getActuation()->get_nu());
  Eigen::VectorXd tau_ub(pw_mpc_test.pw_mpc_->getActuation()->get_nu());
  tau_lb.head(pw_mpc_test.mc_params_->n_rotors_).fill(pw_mpc_test.mc_params_->min_thrust_);
  tau_ub.head(pw_mpc_test.mc_params_->n_rotors_).fill(pw_mpc_test.mc_params_->max_thrust_);
  BOOST_CHECK(pw_mpc_test.mc_params_->n_rotors_ == pw_mpc_test.pw_mpc_->getActuationLowerBounds().size());
  BOOST_CHECK(pw_mpc_test.mc_params_->n_rotors_ == pw_mpc_test.pw_mpc_->getActuationUpperBounds().size());
  BOOST_CHECK(tau_lb == pw_mpc_test.pw_mpc_->getActuationLowerBounds());
  BOOST_CHECK(tau_ub == pw_mpc_test.pw_mpc_->getActuationUpperBounds());

  BOOST_CHECK(pw_mpc_test.pw_mpc_->getStateMultibody()->zero() == pw_mpc_test.pw_mpc_->getInitialState());
  BOOST_CHECK(pw_mpc_test.mc_model_->getFrameId(pw_mpc_test.mc_params_->base_link_name_) ==
              pw_mpc_test.pw_mpc_->getBaseLinkId());

  // Low Level constructors
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission() != nullptr);
  BOOST_CHECK(pw_mpc_test.n_knots_ == pw_mpc_test.pw_mpc_->getKnots());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getTrajectoryGenerator() != nullptr);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission() == pw_mpc_test.mc_mission_);
}

BOOST_AUTO_TEST_CASE(initialize_default_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("mission-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position.fill(1.0);
  w_orientation.fill(1.0);
  w_velocity_lin.fill(1.0);
  w_velocity_ang.fill(1000.0);

  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_running == 1e-6);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_control_running == 1e-4);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_pos_running == 1e-2);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_vel_running == 1e-2);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_pos_terminal == 100);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_vel_terminal == 10);
}

BOOST_AUTO_TEST_CASE(load_parameters_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("mission-test.yaml");

  pw_mpc_test.pw_mpc_->loadParameters(MULTICOPTER_MPC_ROOT_DIR "/unittest/config/ocp/picewise-mpc-test.yaml");

  Eigen::Vector3d w_position;
  Eigen::Vector3d w_orientation;
  Eigen::Vector3d w_velocity_lin;
  Eigen::Vector3d w_velocity_ang;
  w_position << 2.0, 4.0, 1.5;
  w_orientation << 2.0, 2.0, 1.0;
  w_velocity_lin << 1.0, 4.0, 1.0;
  w_velocity_ang << 1.0, 2.0, 1.0;

  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_position == w_position);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_orientation == w_orientation);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_velocity_lin == w_velocity_lin);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_velocity_ang == w_velocity_ang);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_state_running == 1.234);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_control_running == 2.345);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_pos_running == 3.456);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_vel_running == 4.567);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_pos_terminal == 5.678);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getParams().w_vel_terminal == 6.789);
}

BOOST_AUTO_TEST_CASE(initialize_tg_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->initializeTrajectoryGen(multicopter_mpc::SolverTypes::BoxFDDP);

  // Testing the knots assignment once the split has been done
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getProblem() != nullptr);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[0].knots == 100);  // state trajectory 99
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].knots == 101);  // state trajectory 200
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[2].knots == 75);   // state trajectory 274
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[3].knots == 76);   // state trajectory 350

  // Check references of every new WP
  // Position
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[0].pose.translation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(99).head(3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].pose.translation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[0].pose.translation());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[2].pose.translation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(274).head(3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[3].pose.translation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[1].pose.translation());

  // Orientation
  Eigen::Quaterniond quat(
      static_cast<Eigen::Vector4d>(pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(99).segment(3, 4)));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[0].pose.rotation() == quat.toRotationMatrix());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].pose.rotation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[0].pose.rotation());
  quat = Eigen::Quaterniond(
      static_cast<Eigen::Vector4d>(pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(274).segment(3, 4)));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[2].pose.rotation() == quat.toRotationMatrix());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[3].pose.rotation() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[1].pose.rotation());

  // Linear velocity
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[0].vel->linear() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(99).segment(7, 3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].vel->linear() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[0].vel->linear());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[2].vel->linear() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(274).segment(7, 3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[3].vel->linear() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[1].vel->linear());

  // Angular velocity
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[0].vel->angular() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(99).segment(10, 3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].vel->angular() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[0].vel->angular());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[2].vel->angular() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getState(274).segment(10, 3));
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[3].vel->angular() ==
              pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getWaypoints()[1].vel->angular());
}

BOOST_AUTO_TEST_CASE(create_problem_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels().size() == pw_mpc_test.pw_mpc_->getKnots());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getIntegratedRunningModels().size() == pw_mpc_test.pw_mpc_->getKnots());

  // Differential and Integrated->Differential are pointing at the same place
  for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getKnots() - 1; ++i) {
    // Check that the data differential model that std_vector<integrated> is pointing to is the same that the data
    // member differential is pointing to
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            pw_mpc_test.pw_mpc_->getIntegratedRunningModels()[i]);
    BOOST_CHECK(int_model->get_differential() == pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]);
    // Check the action model ptr of every node are pointing to different action model
    if (i < pw_mpc_test.pw_mpc_->getKnots() - 2) {
      BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i] !=
                  pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i + 1]);
      BOOST_CHECK(pw_mpc_test.pw_mpc_->getIntegratedRunningModels()[i] !=
                  pw_mpc_test.pw_mpc_->getIntegratedRunningModels()[i + 1]);
    } else {
      BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i] !=
                  pw_mpc_test.pw_mpc_->getDifferentialTerminalModel());
      BOOST_CHECK(pw_mpc_test.pw_mpc_->getIntegratedRunningModels()[i] !=
                  pw_mpc_test.pw_mpc_->getIntegratedTerminalModel());
    }
  }
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getIntegratedTerminalModel()->get_differential() ==
              pw_mpc_test.pw_mpc_->getDifferentialTerminalModel());

  // Last of running and terminal are pointing at the same place
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getIntegratedTerminalModel() ==
              pw_mpc_test.pw_mpc_->getIntegratedRunningModels().back());
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialTerminalModel() ==
              pw_mpc_test.pw_mpc_->getDifferentialRunningModels().back());
}

BOOST_AUTO_TEST_CASE(create_costs_weights_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getKnots() - 1; ++i) {
    BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("state_reg")
                    ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_state_running);
    BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("control_reg")
                    ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_control_running);
    BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("pose_desired")
                    ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_running);
    BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
                    ->get_costs()
                    ->get_costs()
                    .find("vel_desired")
                    ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_running);
  }
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_terminal);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialTerminalModel()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_terminal);
}

// This test checks whether the reference in the placement cost (from problem) are the same as in the class. This
// should be probably be removed since it might happen that all references are not the same for all the nodes
// BOOST_AUTO_TEST_CASE(create_costs_reference_test, *boost::unit_test::tolerance(1e-7)) {
//   PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

//   pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

//   for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getKnots() - 1; ++i) {
//     boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
//         boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
//             pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
//                 ->get_costs()
//                 ->get_costs()
//                 .find("pose_desired")
//                 ->second->cost);
//     boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
//         boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
//             pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
//                 ->get_costs()
//                 ->get_costs()
//                 .find("vel_desired")
//                 ->second->cost);

//     BOOST_CHECK(cost_pose->get_Mref().oMf == pw_mpc_test.pw_mpc_->getPoseRef().oMf);
//     BOOST_CHECK(cost_pose->get_Mref().frame == pw_mpc_test.pw_mpc_->getPoseRef().frame);
//     BOOST_CHECK(cost_vel->get_vref().oMf == pw_mpc_test.pw_mpc_->getVelocityRef().oMf);
//     BOOST_CHECK(cost_vel->get_vref().frame == pw_mpc_test.pw_mpc_->getVelocityRef().frame);
//   }
//   boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
//       boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
//           pw_mpc_test.pw_mpc_->getDifferentialTerminalModel()
//               ->get_costs()
//               ->get_costs()
//               .find("pose_desired")
//               ->second->cost);
//   boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
//       boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
//           pw_mpc_test.pw_mpc_->getDifferentialTerminalModel()
//               ->get_costs()
//               ->get_costs()
//               .find("vel_desired")
//               ->second->cost);

//   BOOST_CHECK(cost_pose->get_Mref().oMf == pw_mpc_test.pw_mpc_->getPoseRef().oMf);
//   BOOST_CHECK(cost_pose->get_Mref().frame == pw_mpc_test.pw_mpc_->getPoseRef().frame);
//   BOOST_CHECK(cost_vel->get_vref().oMf == pw_mpc_test.pw_mpc_->getVelocityRef().oMf);
//   BOOST_CHECK(cost_vel->get_vref().frame == pw_mpc_test.pw_mpc_->getVelocityRef().frame);
// }

// This checks that the weights for the weighted quadratic barrier in the state costs are properly set
BOOST_AUTO_TEST_CASE(create_cost_state_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getKnots() - 1; ++i) {
    boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
        boost::static_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
            pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
                ->get_costs()
                ->get_costs()
                .find("state_reg")
                ->second->cost->get_activation());
    Eigen::VectorXd weights(pw_mpc_test.pw_mpc_->getParams().w_state_position.size() +
                            pw_mpc_test.pw_mpc_->getParams().w_state_orientation.size() +
                            pw_mpc_test.pw_mpc_->getParams().w_state_velocity_lin.size() +
                            pw_mpc_test.pw_mpc_->getParams().w_state_velocity_ang.size());
    weights << pw_mpc_test.pw_mpc_->getParams().w_state_position, pw_mpc_test.pw_mpc_->getParams().w_state_orientation,
        pw_mpc_test.pw_mpc_->getParams().w_state_velocity_lin, pw_mpc_test.pw_mpc_->getParams().w_state_velocity_ang;
    BOOST_CHECK(activation->get_weights() == weights);
  }
}

// This test is to check that the weights are properly updated when the method updateWeights() is called
BOOST_AUTO_TEST_CASE(update_weights_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  // Before calling the updateProblem() last node should have the terminal weight
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_terminal);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_terminal);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[pw_mpc_test.pw_mpc_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_running);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[pw_mpc_test.pw_mpc_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_running);

  pw_mpc_test.pw_mpc_->updateProblem(pw_mpc_test.pw_mpc_->getKnots());

  // After calling, last node running weights and the penultimate terminal weights
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_running);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()
                  .back()
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_running);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[pw_mpc_test.pw_mpc_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("pose_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_terminal);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[pw_mpc_test.pw_mpc_->getKnots() - 2]
                  ->get_costs()
                  ->get_costs()
                  .find("vel_desired")
                  ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_terminal);
}

// This test is to check that the iterator indicating the terminal weight is the last node after calling
// updateProblem() n_knots - 1 times
BOOST_AUTO_TEST_CASE(update_iterator_cyclic_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  for (int i = 0; i < pw_mpc_test.pw_mpc_->getKnots() - 1; ++i) {
    pw_mpc_test.pw_mpc_->updateProblem(pw_mpc_test.pw_mpc_->getKnots());
  }

  BOOST_CHECK(pw_mpc_test.pw_mpc_->getIterator() == pw_mpc_test.pw_mpc_->getDifferentialRunningModels().end() - 1);
}

BOOST_AUTO_TEST_CASE(update_terminal_cost_test, *boost::unit_test::tolerance(1e-7)) {
  PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  std::size_t trajectory_cursor = pw_mpc_test.pw_mpc_->getKnots();

  pw_mpc_test.pw_mpc_->updateProblem(trajectory_cursor);

  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          pw_mpc_test.pw_mpc_->getDifferentialRunningModels()
              .back()
              ->get_costs()
              ->get_costs()
              .find("pose_desired")
              ->second->cost);
  BOOST_CHECK(cost_pose->get_Mref().oMf == pw_mpc_test.pw_mpc_->getMission()->getWaypoints()[1].pose);
  BOOST_CHECK(pw_mpc_test.pw_mpc_->getHasMotionRef() == true);
}


// The hovering case should be implemented in the updateProblem case
BOOST_AUTO_TEST_CASE(hovering_state_test, *boost::unit_test::tolerance(1e-7)) {
  // PiceWiseMpcTest pw_mpc_test("takeoff.yaml");

  // pw_mpc_test.pw_mpc_->createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  // std::size_t trajectory_cursor = pw_mpc_test.pw_mpc_->getKnots() - 1;

  // for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getTrajectoryGenerator()->getMission()->getTotalKnots() -
  //                                 pw_mpc_test.pw_mpc_->getKnots() + 1;
  //      ++i) {
  //   ++trajectory_cursor;
  //   pw_mpc_test.pw_mpc_->updateProblem(trajectory_cursor);
  // }

  // for (std::size_t i = 0; i < pw_mpc_test.pw_mpc_->getDifferentialRunningModels().size(); ++i) {
  //   boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
  //       boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
  //           pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
  //               ->get_costs()
  //               ->get_costs()
  //               .find("pose_desired")
  //               ->second->cost);
  //   boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
  //       boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
  //           pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
  //               ->get_costs()
  //               ->get_costs()
  //               .find("vel_desired")
  //               ->second->cost);

  //   BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
  //                   ->get_costs()
  //                   ->get_costs()
  //                   .find("pose_desired")
  //                   ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_pos_terminal);
  //   BOOST_CHECK(pw_mpc_test.pw_mpc_->getDifferentialRunningModels()[i]
  //                   ->get_costs()
  //                   ->get_costs()
  //                   .find("vel_desired")
  //                   ->second->weight == pw_mpc_test.pw_mpc_->getParams().w_vel_terminal);
  //   BOOST_CHECK(cost_pose->get_Mref().oMf == pw_mpc_test.pw_mpc_->getMission()->getWaypoints().back().pose);
  //   BOOST_CHECK(cost_vel->get_vref().oMf == pw_mpc_test.pw_mpc_->getMission()->getWaypoints().back().vel);
  // }
}

BOOST_AUTO_TEST_SUITE_END()