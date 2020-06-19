#include "multicopter_mpc/ocp/trajectory-generator-controller.hpp"

namespace multicopter_mpc {

TrajectoryGeneratorController::TrajectoryGeneratorController(const boost::shared_ptr<pinocchio::Model>& model,
                                                             const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                                             const double& dt, const std::size_t& n_knots)
    : OcpAbstract(model, mc_params, dt) {
  n_knots_ = n_knots;

  pos_ref_ = Eigen::Vector3d::Zero();
  quat_ref_ = Eigen::Quaterniond::Identity();
  vel_lin_ref_ = Eigen::Vector3d::Zero();
  vel_ang_ref_ = Eigen::Vector3d::Zero();

  setPoseRef();  
  setMotionRef();

  initializeDefaultParameters();
}

TrajectoryGeneratorController::~TrajectoryGeneratorController() {}

void TrajectoryGeneratorController::initializeDefaultParameters() {
  params_.w_state_position.fill(1.);
  params_.w_state_orientation.fill(1.);
  params_.w_state_velocity_lin.fill(1.);
  params_.w_state_velocity_ang.fill(1000.);

  params_.w_state_running = 1e-6;
  params_.w_control_running = 1e-4;
  params_.w_pos_running = 1e-2;
  params_.w_vel_running = 1e-2;

  params_.w_pos_terminal = 100;
  params_.w_vel_terminal = 10;
}

void TrajectoryGeneratorController::loadParameters(const yaml_parser::ParamsServer& server) {
  std::vector<std::string> state_weights = server.getParam<std::vector<std::string>>("ocp/state_weights");
  std::map<std::string, std::string> current_state =
      yaml_parser::converter<std::map<std::string, std::string>>::convert(state_weights[0]);

  params_.w_state_position = yaml_parser::converter<Eigen::Vector3d>::convert(current_state["position"]);
  params_.w_state_orientation = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["orientation"]);
  params_.w_state_velocity_lin = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_lin"]);
  params_.w_state_velocity_ang = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_ang"]);

  params_.w_state_running = server.getParam<double>("ocp/cost_running_state_weight");
  params_.w_control_running = server.getParam<double>("ocp/cost_running_control_weight");
  params_.w_pos_running = server.getParam<double>("ocp/cost_running_pos_weight");
  params_.w_vel_running = server.getParam<double>("ocp/cost_running_vel_weight");
  params_.w_pos_terminal = server.getParam<double>("ocp/cost_terminal_pos_weight");
  params_.w_vel_terminal = server.getParam<double>("ocp/cost_terminal_vel_weight");
}

void TrajectoryGeneratorController::createProblem(const SolverTypes::Type& solver_type) {
  for (int i = 0; i < n_knots_ - 1; ++i) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = createDifferentialRunningModel();
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

    diff_models_running_.push_back(diff_model);
    int_models_running_.push_back(int_model);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = createDifferentialTerminalModel();
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

  diff_model_terminal_ = diff_model;
  int_model_terminal_ = int_model;

  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(state_initial_, int_models_running_, int_model_terminal_);
  setSolver(solver_type);
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
TrajectoryGeneratorController::createDifferentialRunningModel() {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  // Regularitzations
  cost_model->addCost("state_reg", cost_state, params_.w_state_running);
  cost_model->addCost("control_reg", cost_reg_control, params_.w_control_running);

  // Waypoint cost related
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  cost_model->addCost("pose_desired", cost_pose, params_.w_pos_running);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
      boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());
  cost_model->addCost("vel_desired", cost_vel, params_.w_vel_running);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
TrajectoryGeneratorController::createDifferentialTerminalModel() {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // Waypoint cost related
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  cost_model->addCost("pose_desired", cost_pose, params_.w_pos_terminal);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
      boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());
  cost_model->addCost("vel_desired", cost_vel, params_.w_vel_terminal);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> TrajectoryGeneratorController::createCostStateRegularization() {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3) = params_.w_state_position;
  state_weights.segment(3, 3) = params_.w_state_orientation;
  state_weights.segment(model_->nv, 3) = params_.w_state_velocity_lin;
  state_weights.segment(model_->nv + 3, 3) = params_.w_state_velocity_ang;

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state =
      boost::make_shared<crocoddyl::CostModelState>(state_, activation_state, state_->zero(), actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> TrajectoryGeneratorController::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

void TrajectoryGeneratorController::solve() {
  problem_->set_x0(state_initial_);
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iters_, false, 1e-9);
}

void TrajectoryGeneratorController::updateReferences(const Eigen::Ref<Eigen::VectorXd>& state_new) {
  assert(state_new.size() == state_->get_nx());
  assert(problem_ != nullptr);

  pos_ref_ = state_new.head(3);
  quat_ref_.x() = state_new(3);
  quat_ref_.y() = state_new(4);
  quat_ref_.z() = state_new(5);
  quat_ref_.w() = state_new(6);
  vel_lin_ref_ = state_new.segment(7, 3);
  vel_ang_ref_ = state_new.tail(3);

  setPoseRef();
  setMotionRef();

  // Running models
  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
        boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            diff_models_running_[t]->get_costs()->get_costs().find("pose_desired")->second->cost);
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            diff_models_running_[t]->get_costs()->get_costs().find("vel_desired")->second->cost);
    cost_pose->set_Mref(pose_ref_);
    cost_vel->set_vref(motion_ref_);
  }

  // Terminal models
  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          diff_model_terminal_->get_costs()->get_costs().find("pose_desired")->second->cost);
  boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
      boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
          diff_model_terminal_->get_costs()->get_costs().find("vel_desired")->second->cost);
  cost_pose->set_Mref(pose_ref_);
  cost_vel->set_vref(motion_ref_);
}

const crocoddyl::FramePlacement& TrajectoryGeneratorController::getPoseRef() const { return pose_ref_; }
const crocoddyl::FrameMotion& TrajectoryGeneratorController::getVelocityRef() const { return motion_ref_; }
const TrajectoryGeneratorParams& TrajectoryGeneratorController::getParams() const { return params_; };
const Eigen::VectorXd& TrajectoryGeneratorController::getControls(const std::size_t& idx) const { return solver_->get_us()[idx]; }

void TrajectoryGeneratorController::setPoseRef() {
  pose_ref_.frame = frame_base_link_id_;
  pose_ref_.oMf.translation() = pos_ref_;
  pose_ref_.oMf.rotation() = quat_ref_.toRotationMatrix();
}

void TrajectoryGeneratorController::setMotionRef() {
  motion_ref_.frame = frame_base_link_id_;
  motion_ref_.oMf.linear() = vel_lin_ref_;
  motion_ref_.oMf.angular() = vel_ang_ref_;
}

}  // namespace multicopter_mpc
