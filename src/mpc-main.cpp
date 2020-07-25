#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(const MultiCopterTypes::Type& mc_type, const SolverTypes::Type& solver_type,
                 const std::string& mission_name, const std::string& mpc_type)
    : mc_type_(mc_type), solver_type_(solver_type) {
  std::string model_description_path;
  std::string model_yaml_path;
  std::string mission_yaml_path = MULTICOPTER_MPC_MISSION_DIR "/" + mission_name;
  // std::string llc_params_yaml_path = MULTICOPTER_MPC_OCP_DIR "/trajectory-generator-controller.yaml";
  std::string llc_params_yaml_path = MULTICOPTER_MPC_OCP_DIR "/low-level-controller.yaml";

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

  yaml_parser::ParserYAML yaml_llc_params(llc_params_yaml_path, "", true);
  yaml_parser::ParamsServer server_llc_params(yaml_llc_params.getParams());

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  model_ = boost::make_shared<pinocchio::Model>(model);

  // Multicopter mission and params
  mc_params_ = boost::make_shared<MultiCopterBaseParams>();
  mc_params_->fill(server_params);

  mission_ = boost::make_shared<Mission>(model_->nq + model_->nv);
  mission_->fillWaypoints(server_mission);
  mission_->fillInitialState(server_mission);

  dt_ = 4e-3;

  // Low Level Controller initialization
  mpc_controller_knots_ = 100;
  mpc_controller_ = FactoryMpc::createMpcController(mpc_type, model_, mc_params_, dt_, mission_, mpc_controller_knots_);

  mpc_controller_->loadParameters(server_llc_params);
  current_state_ = mpc_controller_->getStateMultibody()->zero();
  mpc_controller_->setInitialState(current_state_);
  mpc_controller_->createProblem(solver_type_);
  mpc_controller_->setSolverCallbacks(true);
  mpc_controller_->setSolverIters(100);
  mpc_controller_->solve();
  mpc_controller_->setSolverIters(1);
  mpc_controller_->setSolverCallbacks(false);

  current_motor_thrust_ = Eigen::VectorXd::Zero(mpc_controller_->getActuation()->get_nu());
  current_motor_speed_ = current_motor_thrust_;
  // Do check to ensure that the guess of the solver is right

  trajectory_cursor_ = mpc_controller_knots_ - 1;
  std::cout << "MULTICOPTER MPC: MPC Main initialization complete" << std::endl;
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

const boost::shared_ptr<const MpcAbstract> MpcMain::getMpcController() { return mpc_controller_; }

void MpcMain::setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state) { current_state_ = current_state; }

const Eigen::VectorXd& MpcMain::runMpcStep() {
  // 1. update the current state of the low-level-controller
  mpc_controller_->setInitialState(current_state_);
  // 2. solve with the current state
  mpc_controller_->solve();
  // 3. update control variable
  current_motor_thrust_ = mpc_controller_->getControls();
  computeSpeedControls();
  // 4. update low_level->reference trajecotry with the next state from the mpc_main->reference trajectory
  ++trajectory_cursor_;
  mpc_controller_->updateProblem(trajectory_cursor_);
  if (trajectory_cursor_ == mpc_controller_knots_ + mpc_controller_->getMission()->getTotalKnots()) {
    std::cout << "End of trajectory reached. State: \n" << current_state_ << std::endl;
  }

  return current_motor_speed_;
}

void MpcMain::computeSpeedControls() {
  current_motor_speed_ = (current_motor_thrust_.array() / mc_params_->cf_).sqrt();
}
}  // namespace multicopter_mpc