#include "multicopter_mpc/ocp/trajectory-generator.hpp"

#include "multicopter_mpc/utils/log.hpp"
namespace multicopter_mpc {

TrajectoryGenerator::TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                         const boost::shared_ptr<Mission>& mission)
    : OcpAbstract(model, mc_params), mission_(mission) {
  initializeDefaultParameters();

  // To be changed!!!!!
  control_hover_ = Eigen::VectorXd::Ones(mc_params_->n_rotors_) * 3.78;

  barrier_weight_ = 1e-3;
  barrier_quad_weights_aux_ = 0.1 * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
  barrier_quad_weights_ = 1. / barrier_quad_weights_aux_.array().pow(2);
  barrier_act_bounds_ =
      boost::make_shared<crocoddyl::ActivationBounds>(squashing_model_->get_s_lb(), squashing_model_->get_s_ub(), 1.0);
  barrier_activation_ = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(*(barrier_act_bounds_),
                                                                                               barrier_quad_weights_);
  squash_barr_cost_ =
      boost::make_shared<crocoddyl::CostModelControl>(state_, barrier_activation_, squashing_model_->get_ns());
}

TrajectoryGenerator::~TrajectoryGenerator() {}

void TrajectoryGenerator::initializeDefaultParameters() {
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

void TrajectoryGenerator::loadParameters(const std::string& yaml_path) {
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
  params_.w_pos_terminal = server.getParam<double>("ocp/cost_terminal_pos_weight");
  params_.w_vel_terminal = server.getParam<double>("ocp/cost_terminal_vel_weight");
  params_.w_pos_running = server.getParam<double>("ocp/cost_running_pos_weight");
  params_.w_vel_running = server.getParam<double>("ocp/cost_running_vel_weight");

  try {
    dt_ = server.getParam<double>("ocp/dt");
  } catch (const std::exception& e) {
    MMPC_WARN << "TRAJECTORY GENERATOR PARAMS. dt not found, setting default: " << dt_ << '\n';
  }
}

void TrajectoryGenerator::createProblem(const SolverTypes::Type& solver_type,
                                        const IntegratorTypes::Type& integrator_type) {
  assert(dt_ > 0.0);
  assert(n_knots_ == mission_->getTotalKnots() && n_knots_ > 0);

  bool squash = solver_type == SolverTypes::SquashBoxFDDP;

  for (std::vector<WayPoint>::const_iterator wp = mission_->getWaypoints().cbegin();
       wp != mission_->getWaypoints().cend(); ++wp) {
    bool is_last_wp = std::next(wp) == mission_->getWaypoints().end();

    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_running =
        createRunningDifferentialModel(*wp, squash);
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal =
        createTerminalDifferentialModel(*wp, is_last_wp, squash);

    boost::shared_ptr<crocoddyl::ActionModelAbstract> int_model_running;
    boost::shared_ptr<crocoddyl::ActionModelAbstract> int_model_terminal;
    double dt = is_last_wp ? 0.0 : dt_;
    switch (integrator_type_) {
      case IntegratorTypes::Euler:
        int_model_running = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_running, dt_);
        int_model_terminal = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal, dt);
        break;
      case IntegratorTypes::RK4:
        int_model_running = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(diff_model_running, dt_);
        int_model_terminal = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(diff_model_terminal, dt);
        break;
      default:
        int_model_running = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_running, dt_);
        int_model_terminal = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal, dt);
        break;
    }

    int n_run_knots = wp == mission_->getWaypoints().cbegin() ? wp->knots - 1 : wp->knots - 2;

    if (!is_last_wp) {
      int_model_running->set_u_lb(tau_lb_);
      int_model_running->set_u_ub(tau_ub_);
      int_model_terminal->set_u_lb(tau_lb_);
      int_model_terminal->set_u_ub(tau_ub_);

      std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running(
          n_run_knots, diff_model_running);
      diff_models_running.push_back(diff_model_terminal);
      diff_models_running_.insert(diff_models_running_.end(), diff_models_running.begin(), diff_models_running.end());

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running(n_run_knots,
                                                                                        int_model_running);
      int_models_running.push_back(int_model_terminal);
      int_models_running_.insert(int_models_running_.end(), int_models_running.begin(), int_models_running.end());
    } else {
      int_model_running->set_u_lb(tau_lb_);
      int_model_running->set_u_ub(tau_ub_);

      std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running(
          n_run_knots, diff_model_running);
      diff_models_running_.insert(diff_models_running_.end(), diff_models_running.begin(), diff_models_running.end());
      diff_model_terminal_ = diff_model_terminal;

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running(n_run_knots,
                                                                                        int_model_running);
      int_models_running_.insert(int_models_running_.end(), int_models_running.begin(), int_models_running.end());
      int_model_terminal_ = int_model_terminal;
    }
  }
  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(mission_->getInitialState(), int_models_running_,
                                                            int_model_terminal_);

  setSolver(solver_type_);
  setInitialState(mission_->getInitialState());
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
TrajectoryGenerator::createRunningDifferentialModel(const WayPoint& waypoint, const bool& squash) {
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  if (squash) {
    actuation = actuation_squashed_;
  } else {
    actuation = actuation_;
  }

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  boost::shared_ptr<crocoddyl::CostModelSum> cost_model_running =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation->get_nu());

  // Regularitzations
  cost_model_running->addCost("state_reg", cost_reg_state, params_.w_state_running);
  cost_model_running->addCost("control_reg", cost_reg_control, params_.w_control_running);
  if (squash) {
    cost_model_running->addCost("barrier", squash_barr_cost_, barrier_weight_);
  }

  // Diff & Integrated models
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_running =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation, cost_model_running);

  return diff_model_running;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
TrajectoryGenerator::createTerminalDifferentialModel(const WayPoint& waypoint, const bool& is_last_wp,
                                                     const bool& squash) {
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  if (squash) {
    actuation = actuation_squashed_;
  } else {
    actuation = actuation_;
  }

  // Regularitzations
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  boost::shared_ptr<crocoddyl::CostModelSum> cost_model_terminal =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation->get_nu());

  double w_pos = params_.w_pos_terminal;
  double w_vel = params_.w_vel_terminal;

  if (!is_last_wp) {
    cost_model_terminal->addCost("state_reg", cost_reg_state, params_.w_state_running);
    cost_model_terminal->addCost("control_reg", cost_reg_control, params_.w_control_running);
    if (squash) {
      cost_model_terminal->addCost("barrier", squash_barr_cost_, barrier_weight_);
    }

    w_pos = params_.w_pos_terminal / dt_;
    w_vel = params_.w_vel_terminal / dt_;
  }

  // Waypoint cost related
  crocoddyl::FramePlacement frame_ref = crocoddyl::FramePlacement(frame_base_link_id_, waypoint.pose);
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, frame_ref, actuation->get_nu());
  cost_model_terminal->addCost("pos_desired", cost_goal, w_pos);

  if (waypoint.vel != boost::none) {
    crocoddyl::FrameMotion vel_ref(frame_base_link_id_, *(waypoint.vel));
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal_vel =
        boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, vel_ref, actuation->get_nu());
    cost_model_terminal->addCost("vel_desired", cost_goal_vel, w_vel);
  }

  // Diff & Integrated models
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation, cost_model_terminal);

  return diff_model_terminal;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> TrajectoryGenerator::createCostStateRegularization() {
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

boost::shared_ptr<crocoddyl::CostModelAbstract> TrajectoryGenerator::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

void TrajectoryGenerator::setTimeStep(const double& dt) {
  dt_ = dt;
  mission_->setTimeStep(dt);
  n_knots_ = mission_->getTotalKnots();

  if (problem_ != nullptr) {
    diff_models_running_.clear();
    diff_model_terminal_ = nullptr;

    int_models_running_.clear();
    int_model_terminal_ = nullptr;

    problem_ = nullptr;
    solver_ = nullptr;
    // Notice that when changing the dt, create problem should be called again!
  }
}

void TrajectoryGenerator::solve(const std::vector<Eigen::VectorXd>& state_trajectory,
                                const std::vector<Eigen::VectorXd>& control_trajectory) {
  OcpAbstract::solve(state_trajectory, control_trajectory);
  setStateHover();
}

void TrajectoryGenerator::setStateHover() {
  assert(solver_->get_xs().size() > 0);

  state_hover_ = state_->zero();
  state_hover_.head(3) = solver_->get_xs().back().head(3);
  Eigen::Quaterniond quat = Eigen::Quaterniond(solver_->get_xs().back()(6), 0.0, 0.0, solver_->get_xs().back()(5));
  quat.normalize();
  state_hover_(5) = quat.z();
  state_hover_(6) = quat.w();
}

const boost::shared_ptr<Mission> TrajectoryGenerator::getMission() const { return mission_; }

const TrajectoryGeneratorParams& TrajectoryGenerator::getParams() const { return params_; };

std::vector<Eigen::VectorXd> TrajectoryGenerator::getStateTrajectory(const std::size_t& idx_init,
                                                                     const std::size_t& idx_end) const {
  assert(idx_init < idx_end);
  std::vector<Eigen::VectorXd>::const_iterator first = solver_->get_xs().begin() + idx_init;
  std::vector<Eigen::VectorXd>::const_iterator last = solver_->get_xs().begin() + idx_end + 1;
  return std::vector<Eigen::VectorXd>(first, last);
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::getControlTrajectory(const std::size_t& idx_init,
                                                                       const std::size_t& idx_end) const {
  assert(idx_init < idx_end);
  std::vector<Eigen::VectorXd>::const_iterator first = controls_.begin() + idx_init;
  std::vector<Eigen::VectorXd>::const_iterator last = controls_.begin() + idx_end + 1;
  return std::vector<Eigen::VectorXd>(first, last);
}

const Eigen::VectorXd& TrajectoryGenerator::getState(const std::size_t& cursor) const {
  if (cursor < states_.size()) {
    return states_[cursor];
  } else {
    return state_hover_;
  }
}

const Eigen::VectorXd& TrajectoryGenerator::getControl(const std::size_t& cursor) const {
  if (cursor < controls_.size()) {
    return controls_[cursor];
  } else {
    return control_hover_;
  }
}

}  // namespace multicopter_mpc