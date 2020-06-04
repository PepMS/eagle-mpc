#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(MultiCopterTypes::Type mc_type, SolverTypes::Type solver_type)
    : mc_type_(mc_type), solver_type_(solver_type) {
  std::string model_description_path;
  std::string model_yaml_path;

  switch (mc_type_) {
    case MultiCopterTypes::Iris:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
      model_yaml_path = MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml";
      break;
    case MultiCopterTypes::Hector:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf";
      model_yaml_path = MULTICOPTER_MPC_MULTIROTOR_DIR "/hector.yaml";
      break;
    default:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
      model_yaml_path = MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml";
      break;
  }

  yaml_parser::ParserYAML yaml_mc(model_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_mc.getParams());

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  model_ = boost::make_shared<pinocchio::Model>(model);

  // Multicopter mission and params
  mc_params_ = boost::make_shared<MultiCopterBaseParams>();
  mc_params_->fill(server_params);

  dt_ = 1e-2;
  low_level_controller_knots_ = 100;
  low_level_controller_ = boost::make_shared<LowLevelController>(model_, mc_params_, dt_, low_level_controller_knots_);

  // HOVER test purposes
  Eigen::Quaterniond quat(1, 0, 0, 1);
  quat.normalize();
  current_motor_thrust_ = Eigen::VectorXd::Zero(low_level_controller_->getActuation()->get_nu());
  current_motor_speed_ = current_motor_thrust_;
  current_state_ = low_level_controller_->getState()->zero();
  low_level_controller_->setInitialState(current_state_);
  current_state_(2) = 0.3;
  // current_state_(5) = quat.z();
  // current_state_(6) = quat.w();
  std::vector<Eigen::VectorXd> reference_trajectory(low_level_controller_->getKnots(), current_state_);
  low_level_controller_->setReferenceStateTrajectory(reference_trajectory);
  low_level_controller_->createProblem(solver_type_);
  low_level_controller_->setSolverCallbacks(true);
  low_level_controller_->setSolverIters(100);
  low_level_controller_->solve();
  // set low level controller for continuous mode
  low_level_controller_->setSolverIters(1);
  low_level_controller_->setSolverCallbacks(true);
  // Do check to ensure that the guess of the solver is right
  std::cout << "MULTICOPTER MPC: MPC Main initialization complete" << std::endl;
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

const boost::shared_ptr<const LowLevelController> MpcMain::getLowLevelController() { return low_level_controller_; }

void MpcMain::setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state) { current_state_ = current_state; }

const Eigen::VectorXd& MpcMain::runMpcStep() {
  // 1. update the current state of the low-level-controller
  low_level_controller_->setInitialState(current_state_);
  // 2. solve with the current state
  low_level_controller_->solve();
  // 3. update control variable
  current_motor_thrust_ = low_level_controller_->getControls();
  computeSpeedControls();
  // 4. update low_level->reference trajecotry with the next state from the mpc_main->reference trajectory
  // TBD (not needed in the hovering case)
  return current_motor_speed_;
}

void MpcMain::computeSpeedControls() {
  current_motor_speed_ = (current_motor_thrust_.array() / mc_params_->cf_).sqrt();
  // current_motor_speed_ = MOTOR_SPEED_MIN + (current_motor_thrust_.array() - mc_params_->min_thrust_) /
  //                                              (mc_params_->max_thrust_ - mc_params_->min_thrust_) *
  //                                              (MOTOR_SPEED_MAX - MOTOR_SPEED_MIN);
}
}  // namespace multicopter_mpc

// const Eigen::VectorXd& MpcMain::getActuatorControls() const { return controls_; }
// const Eigen::VectorXd& MpcMain::getActuatorControlsNormalized() const { return controls_normalized_; }

// void MpcMain::computeNormalizedControls() {
//   controls_normalized_ = MOTOR_TH_NORM_MIN + (controls_.array() - params_->min_thrust_) /
//                                                  (params_->max_thrust_ - params_->min_thrust_) *
//                                                  (MOTOR_TH_NORM_MAX - MOTOR_TH_NORM_MIN);
// }