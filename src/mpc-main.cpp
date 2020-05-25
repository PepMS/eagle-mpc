#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(MultiCopterTypes::Type mc_type, MissionTypes::Type mission_type, SolverTypes::Type solver_type)
    : mc_type_(mc_type), mission_type_(mission_type), solver_type_(solver_type) {
  std::string model_description_path;
  std::string model_yaml_path;
  std::string model_frame_name;

  std::string mission_yaml_path;

  switch (mc_type_) {
    case MultiCopterTypes::Iris:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
      model_yaml_path = "/usr/local/share/multicopter_mpc/multirotor/iris.yaml";
      model_frame_name = "iris__base_link";
      break;
    case MultiCopterTypes::Hector:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf";
      model_yaml_path = "/usr/local/share/multicopter_mpc/multirotor/hector.yaml";
      model_frame_name = "base_link";
      break;
    default:
      model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
      model_yaml_path = "/usr/local/share/multicopter_mpc/multirotor/iris.yaml";
      model_frame_name = "iris__base_link";
      break;
  }

  switch (mission_type_) {
    case MissionTypes::Hover:
      mission_yaml_path = "/usr/local/share/multicopter_mpc/mission/hover.yaml";
      break;
    case MissionTypes::TakeOff:
      mission_yaml_path = "/usr/local/share/multicopter_mpc/mission/takeoff.yaml";
      break;
    default:
      break;
  }

  yaml_parser::ParserYAML yaml_mc(model_yaml_path, "", true);
  yaml_parser::ParamsServer server_params(yaml_mc.getParams());

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  model_ = boost::make_shared<pinocchio::Model>(model);

  params_ = boost::make_shared<MultiCopterBaseParams>();
  params_->fill(server_params);
  mission_ = boost::make_shared<Mission>(model_->nq + model_->nv);
  mission_->fillWaypoints(server_mission);
  mission_->fillInitialState(server_mission);  // should be changed with every solving iteration

  state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);
  dt_ = 4e-3;
  switch (solver_type_) {
    case SolverTypes::BoxFDDP: {
      problem_ = boost::make_shared<ProblemMission>(
          mission_, params_, model_,
          boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state_, params_->n_rotors_, params_->tau_f_),
          model_->getFrameId(model_frame_name), dt_);
      problem_opt_ = problem_->createProblem();
      solver_ = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem_opt_);
      break;
    }
    case SolverTypes::SquashBoxFDDP: {
      break;
    }
    default:
      break;
  }

  solver_->solve();
  std::cout << "MULTICOPTER MPC: MPC Main initialization complete" << std::endl;

  controls_normalized_ = Eigen::Vector3d::Zero(params_->n_rotors_);
  controls_ = Eigen::Vector3d::Zero(params_->n_rotors_);
}

MpcMain::MpcMain() {}

MpcMain::~MpcMain() {}

void MpcMain::solve() {
  solver_->solve(solver_->get_xs(), solver_->get_us(), 1);
  controls_ = solver_->get_us().front();
  computeNormalizedControls();
}

void MpcMain::setInitialState(const Eigen::Ref<const Eigen::VectorXd>& initial_state) {
  // TODO: Check dimension!!!
  problem_opt_->set_x0(initial_state);
}

const Eigen::VectorXd& MpcMain::getState(const size_t& n_node) const {
  // Check problem number of nodes!
  return solver_->get_xs()[n_node];
}

const Eigen::VectorXd& MpcMain::getActuatorControls() const { return controls_; }
const Eigen::VectorXd& MpcMain::getActuatorControlsNormalized() const { return controls_normalized_; }

void MpcMain::computeNormalizedControls() {
  controls_normalized_ = MOTOR_TH_NORM_MIN + (controls_.array() - params_->min_thrust_) /
                                                 (params_->max_thrust_ - params_->min_thrust_) *
                                                 (MOTOR_TH_NORM_MAX - MOTOR_TH_NORM_MIN);
}

}  // namespace multicopter_mpc