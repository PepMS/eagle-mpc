#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(const MultiCopterTypes::Type& mc_type, const SolverTypes::Type& solver_type,
                 const std::string& mission_name, const std::string& mpc_type, const double& dt)
    : mc_type_(mc_type), solver_type_(solver_type) {
  std::string model_description_path;
  std::string model_yaml_path;
  std::string mission_yaml_path = MULTICOPTER_MPC_MISSION_DIR "/" + mission_name;

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

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  model_ = boost::make_shared<pinocchio::Model>(model);

  // Multicopter mission and params
  mc_params_ = boost::make_shared<MultiCopterBaseParams>();
  mc_params_->fill(server_params);

  mission_ = boost::make_shared<Mission>(model_->nq + model_->nv);
  mission_->fillWaypoints(server_mission);
  mission_->fillInitialState(server_mission);

  dt_ = dt;

  // Low Level Controller initialization
  mpc_controller_knots_ = 100;
  mpc_controller_ =
      FactoryMpc::get().createMpcController(mpc_type, model_, mc_params_, dt_, mission_, mpc_controller_knots_);
  mpc_controller_->loadParameters(mpc_controller_->getParametersPath());
  state_ = mpc_controller_->getStateMultibody()->zero();
  mpc_controller_->setInitialState(state_);
  mpc_controller_->createProblem(solver_type_);
  mpc_controller_->setSolverCallbacks(true);
  mpc_controller_->setSolverIters(100);
  mpc_controller_->solve();
  mpc_controller_->setSolverIters(1);
  mpc_controller_->setSolverCallbacks(false);

  motor_thrust_ = Eigen::VectorXd::Zero(mpc_controller_->getActuation()->get_nu());
  motor_speed_ = motor_thrust_;

  ff_gains_ = Eigen::VectorXd::Zero(mpc_controller_->getStateMultibody()->get_ndx());
  fb_gains_ = Eigen::MatrixXd::Zero(mpc_controller_->getActuation()->get_nu(),
                                            mpc_controller_->getStateMultibody()->get_ndx());

  // Do check to ensure that the guess of the solver is right
  trajectory_cursor_ = mpc_controller_knots_ - 1;
  std::cout << "MULTICOPTER MPC: MPC Main initialization complete" << std::endl;
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

void MpcMain::runMpcStep() {
  // 1. update the current state
  mpc_controller_->setInitialState(state_);
  // 2. solve with the current state
  mpc_controller_->solve();
  // 3. update control variable
  motor_thrust_ = mpc_controller_->getControls();
  thrustToSpeed(motor_thrust_, motor_speed_);
  ff_gains_ = mpc_controller_->getFeedForwardGains();
  fb_gains_ = mpc_controller_->getFeedBackGains();
  // 4. update problem
  ++trajectory_cursor_;
  mpc_controller_->updateProblem(trajectory_cursor_);

  // Print state when reached the final knot
  if (trajectory_cursor_ == mpc_controller_knots_ + mpc_controller_->getMission()->getTotalKnots()) {
    std::cout << "End of trajectory reached. State: \n" << state_ << std::endl;
  }
}

void MpcMain::thrustToSpeed(const Eigen::Ref<const Eigen::VectorXd>& motors_thrust,
                            Eigen::Ref<Eigen::VectorXd> motors_speed) {
  motors_speed = (motors_thrust.array() / mc_params_->cf_).sqrt();
}

void MpcMain::setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state) { state_ = current_state; }

const boost::shared_ptr<const MpcAbstract> MpcMain::getMpcController() { return mpc_controller_; }

const Eigen::VectorXd& MpcMain::getMotorsSpeed() { return motor_speed_; }

const Eigen::VectorXd& MpcMain::getMotorsThrust() { return motor_thrust_; }

const Eigen::VectorXd& MpcMain::getFeedForwardGains() { return ff_gains_; }

const Eigen::MatrixXd& MpcMain::getFeedBackGains() { return fb_gains_; }

void MpcMain::getStateDiff(const Eigen::Ref<const Eigen::VectorXd>& state0,
                           const Eigen::Ref<const Eigen::VectorXd>& state1, Eigen::Ref<Eigen::VectorXd> state_diff) {
  mpc_controller_->getStateMultibody()->diff(state0, state1, state_diff);
}

}  // namespace multicopter_mpc