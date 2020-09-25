#include "multicopter_mpc/mpc-main.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(const MultiCopterTypes::Type& mc_type, const std::string& mission_name,
                 const std::string& mpc_yaml_path)
    : mc_type_(mc_type) {
  std::string model_description_path;
  std::string model_yaml_path;
  std::string mission_yaml_path = MULTICOPTER_MPC_MISSION_DIR "/" + mission_name;
  MMPC_INFO << "MULTICOPTER MPC: MPC Main initialization complete";

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

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  model_ = boost::make_shared<pinocchio::Model>(model);

  // Multicopter mission and params
  mc_params_ = boost::make_shared<MultiCopterBaseParams>();
  mc_params_->fill(model_yaml_path);

  mission_ = boost::make_shared<Mission>(model_->nq + model_->nv);
  mission_->fillWaypoints(mission_yaml_path);

  loadParameters(mpc_yaml_path);
  initializeMpcController();
  state_trajectory_ = mpc_controller_->getSolver()->get_xs();
  control_trajectory_ = mpc_controller_->getSolver()->get_us();

  motor_thrust_ = Eigen::VectorXd::Zero(mpc_controller_->getActuation()->get_nu());
  motor_speed_ = motor_thrust_;

  ff_gains_ = Eigen::VectorXd::Zero(mpc_controller_->getStateMultibody()->get_ndx());
  fb_gains_ = Eigen::MatrixXd::Zero(mpc_controller_->getActuation()->get_nu(),
                                    mpc_controller_->getStateMultibody()->get_ndx());

  // Do check to ensure that the guess of the solver is right
  trajectory_cursor_ = mpc_controller_->getKnots() - 1;
  MMPC_INFO << "MULTICOPTER MPC: MPC Main initialization complete";
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

void MpcMain::loadParameters(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

  mpc_controller_specs_.type = server.getParam<std::string>("mpc_controller/type");
  mpc_controller_specs_.yaml_path = server.getParam<std::string>("mpc_controller/yaml_path");
  if (server.getParam<std::string>("mpc_controller/solver") == "BoxFDDP") {
    mpc_controller_specs_.solver = SolverTypes::BoxFDDP;
  } else if (server.getParam<std::string>("mpc_controller/solver") == "BoxDDP") {
    mpc_controller_specs_.solver = SolverTypes::BoxDDP;
  }
  if (server.getParam<std::string>("mpc_controller/integrator") == "Euler") {
    mpc_controller_specs_.integrator = IntegratorTypes::Euler;
  } else if (server.getParam<std::string>("mpc_controller/integrator") == "RK4") {
    mpc_controller_specs_.integrator = IntegratorTypes::RK4;
  }
  try {
    mpc_controller_specs_.running_iters = server.getParam<int>("mpc_controller/running_iters");
    MMPC_INFO << "MPC CONTROLLER PARAMS. running_iters set to: " << mpc_controller_specs_.running_iters;
  } catch (const std::exception& e) {
    mpc_controller_specs_.running_iters = 1;
    MMPC_WARN << "MPC CONTROLLER PARAMS. running_iters not found, set to default: "
              << mpc_controller_specs_.running_iters;
  }
  try {
    mpc_controller_specs_.callback = server.getParam<bool>("mpc_controller/callback");
  } catch (const std::exception& e) {
    mpc_controller_specs_.callback = false;
  }
}

void MpcMain::initializeMpcController() {
  mpc_controller_ = FactoryMpc::get().createMpcController(mpc_controller_specs_.type, model_, mc_params_, mission_);
  mpc_controller_->loadParameters(mpc_controller_specs_.yaml_path);

  state_ = mpc_controller_->getStateMultibody()->zero();
  mpc_controller_->setInitialState(state_);
  mpc_controller_->createProblem(mpc_controller_specs_.solver, mpc_controller_specs_.integrator,
                                 mpc_controller_->getTimeStep());
  mpc_controller_->setSolverCallbacks(true);
  mpc_controller_->setSolverIters(100);
  mpc_controller_->solve(
      mpc_controller_->getTrajectoryGenerator()->getStateTrajectory(0, mpc_controller_->getKnots() - 1),
      mpc_controller_->getTrajectoryGenerator()->getControlTrajectory(0, mpc_controller_->getKnots() - 2));
  mpc_controller_->setSolverIters(mpc_controller_specs_.running_iters);
  mpc_controller_->setSolverCallbacks(mpc_controller_specs_.callback);
}

void MpcMain::runMpcStep() {
  mpc_controller_->setInitialState(state_);
  mpc_controller_->solve(state_trajectory_, control_trajectory_);

  std::copy(mpc_controller_->getSolver()->get_xs().begin() + 1, mpc_controller_->getSolver()->get_xs().end(),
            state_trajectory_.begin());
  std::copy(mpc_controller_->getSolver()->get_us().begin() + 1, mpc_controller_->getSolver()->get_us().end(),
            control_trajectory_.begin());
  motor_thrust_ = mpc_controller_->getControls(1);
  thrustToSpeed(motor_thrust_, motor_speed_);
  ff_gains_ = mpc_controller_->getFeedForwardGains(1);
  fb_gains_ = mpc_controller_->getFeedBackGains(1);

  ++trajectory_cursor_;
  state_trajectory_.back() = mpc_controller_->getTrajectoryGenerator()->getState(trajectory_cursor_);
  control_trajectory_.back() = *(control_trajectory_.end() - 2);
  mpc_controller_->updateProblem(trajectory_cursor_);
}

void MpcMain::thrustToSpeed(const Eigen::Ref<const Eigen::VectorXd>& motors_thrust,
                            Eigen::Ref<Eigen::VectorXd> motors_speed) {
  motors_speed = (motors_thrust.array() / mc_params_->cf_).sqrt();
}

void MpcMain::setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state) { state_ = current_state; }

const boost::shared_ptr<const MpcAbstract> MpcMain::getMpcController() { return mpc_controller_; }
const Eigen::VectorXd& MpcMain::getCurrentState() { return state_; }
const Eigen::VectorXd& MpcMain::getMotorsSpeed() { return motor_speed_; }
const Eigen::VectorXd& MpcMain::getMotorsThrust() { return motor_thrust_; }
const Eigen::VectorXd& MpcMain::getFeedForwardGains() { return ff_gains_; }
const Eigen::MatrixXd& MpcMain::getFeedBackGains() { return fb_gains_; }

void MpcMain::getStateDiff(const Eigen::Ref<const Eigen::VectorXd>& state0,
                           const Eigen::Ref<const Eigen::VectorXd>& state1, Eigen::Ref<Eigen::VectorXd> state_diff) {
  mpc_controller_->getStateMultibody()->diff(state0, state1, state_diff);
}

}  // namespace multicopter_mpc