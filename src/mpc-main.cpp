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

  std::cout << "MULTICOPTER MPC: MPC Main initialization complete" << std::endl;
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

// const Eigen::VectorXd& MpcMain::getState(const size_t& n_node) const {
//   // Check problem number of nodes!
//   return solver_->get_xs()[n_node];
// }

// const Eigen::VectorXd& MpcMain::getActuatorControls() const { return controls_; }
// const Eigen::VectorXd& MpcMain::getActuatorControlsNormalized() const { return controls_normalized_; }

// void MpcMain::computeNormalizedControls() {
//   controls_normalized_ = MOTOR_TH_NORM_MIN + (controls_.array() - params_->min_thrust_) /
//                                                  (params_->max_thrust_ - params_->min_thrust_) *
//                                                  (MOTOR_TH_NORM_MAX - MOTOR_TH_NORM_MIN);
// }

}  // namespace multicopter_mpc