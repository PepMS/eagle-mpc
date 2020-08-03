#include "multicopter_mpc/ocp/mpc/carrot-mpc.hpp"

namespace multicopter_mpc {

CarrotMpc::CarrotMpc(const boost::shared_ptr<pinocchio::Model>& model,
                     const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                     const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots)
    : MpcAbstract(model, mc_params, dt, mission, n_knots) {
  initializeDefaultParameters();
  parameters_yaml_path_ = MULTICOPTER_MPC_OCP_DIR "/carrot-mpc.yaml";
  terminal_weights_idx_ = std::vector<bool>(n_knots_, false);
}

CarrotMpc::~CarrotMpc() {}

std::string CarrotMpc::getFactoryName() { return "CarrotMpc"; }

boost::shared_ptr<MpcAbstract> CarrotMpc::createMpcController(
    const boost::shared_ptr<pinocchio::Model>& model, const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
    const double& dt, const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots) {
  return boost::make_shared<CarrotMpc>(model, mc_params, dt, mission, n_knots);
}

bool CarrotMpc::registered_ =
    FactoryMpc::get().registerMpcController(CarrotMpc::getFactoryName(), CarrotMpc::createMpcController);

void CarrotMpc::initializeDefaultParameters() {
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

void CarrotMpc::loadParameters(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

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

void CarrotMpc::initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) {
  has_motion_ref_ = false;

  trajectory_generator_->createProblem(solver_type);
  trajectory_generator_->setSolverIters(300);
  trajectory_generator_->solve();

  mission_ = trajectory_generator_->getMission();
}

void CarrotMpc::initializeTerminalWeights() {
  std::size_t last_wp_idx = 0;
  while (last_wp_idx < mission_->getWpTrajIdx().size() && n_knots_ - 1 >= mission_->getWpTrajIdx()[last_wp_idx]) {
    terminal_weights_idx_[mission_->getWpTrajIdx()[last_wp_idx]] = true;
    ++last_wp_idx;
  }

  // If the whole MPC horizon is longer than the mission, the tail of the horizon will be all terminal nodes
  if (last_wp_idx >= mission_->getWpTrajIdx().size()) {
    for (std::size_t i = mission_->getWpTrajIdx()[last_wp_idx - 1]; i < n_knots_; ++i) {
      terminal_weights_idx_[i] = true;
    }
  }
}

const bool CarrotMpc::existsTerminalWeight() {
  return std::find(terminal_weights_idx_.begin(), terminal_weights_idx_.end(), 1) != terminal_weights_idx_.end();
}

void CarrotMpc::createProblem(const SolverTypes::Type& solver_type) {
  initializeTrajectoryGenerator(solver_type);
  initializeTerminalWeights();

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      createDifferentialModel(n_knots_ - 1);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

  diff_model_terminal_ = diff_model;
  int_model_terminal_ = int_model;

  for (int i = n_knots_ - 2; i >= 0; --i) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = createDifferentialModel(i);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);
    diff_models_running_.insert(diff_models_running_.begin(), diff_model);
    int_models_running_.insert(int_models_running_.begin(), int_model);
  }

  // Check that the running differential model from the problem DOES NOT contain the terminal model
  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(state_initial_, int_models_running_, int_model_terminal_);

  // Here check in unittest that terminal and last item in vector are pointing at the same place
  // check also that the size of the vectors are equal to n_knots_
  diff_models_running_.push_back(diff_model_terminal_);
  int_models_running_.push_back(int_model_terminal_);

  setSolver(solver_type);

  diff_model_iter_ = diff_models_running_.end() - 1;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> CarrotMpc::createDifferentialModel(
    const std::size_t& idx_knot) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  // Regularitzations
  cost_model->addCost("state_reg", cost_reg_state, params_.w_state_running);
  cost_model->addCost("control_reg", cost_reg_control, params_.w_control_running);

  // Set reference if it is the last node or a WayPoint node
  if (idx_knot == n_knots_ - 1 || terminal_weights_idx_[idx_knot]) {
    setReference(idx_knot);
  }

  double weight_pos = 0.0;
  double weight_vel = 0.0;

  if (terminal_weights_idx_[idx_knot] || (idx_knot == n_knots_ - 1 && !existsTerminalWeight())) {
    weight_pos = params_.w_pos_terminal;
    weight_vel = params_.w_vel_terminal;
  } else {
    weight_pos = params_.w_pos_running;
    weight_vel = params_.w_vel_running;
  }

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  cost_model->addCost("pose_desired", cost_pose, weight_pos);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
      boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());
  cost_model->addCost("vel_desired", cost_vel, weight_vel);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CarrotMpc::createCostStateRegularization() {
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

boost::shared_ptr<crocoddyl::CostModelAbstract> CarrotMpc::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

void CarrotMpc::solve() {
  problem_->set_x0(state_initial_);
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iters_, false, 1e-9);
}

void CarrotMpc::updateProblem(const std::size_t idx_trajectory) {
  // first update the terminal weight vector
  std::rotate(terminal_weights_idx_.begin(), terminal_weights_idx_.begin() + 1, terminal_weights_idx_.end());
  terminal_weights_idx_.back() = std::find(mission_->getWpTrajIdx().begin(), mission_->getWpTrajIdx().end(),
                                           idx_trajectory) != mission_->getWpTrajIdx().end();

  // Treat the incoming knot
  setReference(idx_trajectory);
  diff_model_iter_ = diff_models_running_.end() - 1;
  // -- References
  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->cost);
  boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
      boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
          (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->cost);
  cost_pose->set_Mref(pose_ref_);
  cost_vel->set_vref(motion_ref_);
  // -- Weights
  if (existsTerminalWeight()) {
    (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_running;
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_running;
  } else {
    (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_terminal;
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_terminal;
  }
  --diff_model_iter_;

  // Treat the subsequent knots
  bool first_terminal_found = false;
  for (std::size_t i = 0; i < n_knots_ - 1; ++i) {
    if (terminal_weights_idx_[n_knots_ - 2 - i]) {
      setReference(idx_trajectory - i - 1);
      (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_terminal;
      (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_terminal;
    } else {
      (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_running;
      (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_running;
    }

    cost_pose = boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
        (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->cost);
    cost_vel = boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
        (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->cost);
    cost_pose->set_Mref(pose_ref_);
    cost_vel->set_vref(motion_ref_);

    --diff_model_iter_;
  }
  // Increase the weight for the next node
}

const crocoddyl::FramePlacement& CarrotMpc::getPoseRef() const { return pose_ref_; }
const crocoddyl::FrameMotion& CarrotMpc::getVelocityRef() const { return motion_ref_; }
const TrajectoryGeneratorParams& CarrotMpc::getParams() const { return params_; };

void CarrotMpc::setReference(const std::size_t& idx_trajectory) {
  state_ref_ = trajectory_generator_->getState(idx_trajectory);
  quat_ref_ = Eigen::Quaterniond(static_cast<Eigen::Vector4d>(state_ref_.segment(3, 4)));

  pose_ref_.frame = frame_base_link_id_;
  pose_ref_.oMf = pinocchio::SE3(quat_ref_.matrix(), static_cast<Eigen::Vector3d>(state_ref_.head(3)));
  motion_ref_.frame = frame_base_link_id_;
  motion_ref_.oMf = pinocchio::Motion(static_cast<Eigen::Vector3d>(state_ref_.segment(7, 3)),
                                      static_cast<Eigen::Vector3d>(state_ref_.segment(10, 3)));
}

}  // namespace multicopter_mpc