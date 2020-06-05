#include "multicopter_mpc/ocp/trajectory-generator.hpp"

namespace multicopter_mpc {

TrajectoryGenerator::TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                         const boost::shared_ptr<Mission>& mission)
    : OcpAbstract(model, mc_params, dt), mission_(mission) {}

TrajectoryGenerator::~TrajectoryGenerator() {}

void TrajectoryGenerator::createProblem(const SolverTypes::Type& solver_type) {
  assert(mission_->waypoints_.size() > 0);

  n_knots_ = mission_->getTotalKnots();

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  for (std::vector<WayPoint>::const_iterator wp = mission_->waypoints_.begin(); wp != mission_->waypoints_.end();
       ++wp) {
    boost::shared_ptr<crocoddyl::CostModelSum> cost_model_running =
        boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
    boost::shared_ptr<crocoddyl::CostModelSum> cost_model_terminal =
        boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

    // Regularitzations
    cost_model_running->addCost("x_reg", cost_reg_state, 1e-6);
    cost_model_running->addCost("u_reg", cost_reg_control, 1e-4);  // 1e-4
    if (std::next(wp) != mission_->waypoints_.end()) {
      cost_model_terminal->addCost("x_reg_cost", cost_reg_state, 1e-6);
      cost_model_terminal->addCost("u_reg_cost", cost_reg_control, 1e-4);  // 1e-4
    }

    // Waypoint cost related
    crocoddyl::FramePlacement frame_ref = crocoddyl::FramePlacement(frame_base_link_id_, wp->pose);
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal =
        boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, frame_ref, actuation_->get_nu());
    cost_model_running->addCost("track_pos_cost", cost_goal, 1e-2);
    cost_model_terminal->addCost("goal_pos_cost", cost_goal, 100);

    if (wp->vel != boost::none) {
      crocoddyl::FrameMotion vel_ref(frame_base_link_id_, *(wp->vel));
      boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal_vel =
          boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, vel_ref, actuation_->get_nu());
      cost_model_running->addCost("track_vel_cost", cost_goal_vel, 1e-2);
      cost_model_terminal->addCost("goal_vel_cost", cost_goal_vel, 10);
    }

    // Diff & Integrated models
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_running =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model_running);
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model_terminal);

    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_running =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_running, dt_);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_terminal =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal, dt_);

    if (std::next(wp) != mission_->waypoints_.end()) {
      int_model_running->set_u_lb(tau_lb_);
      int_model_running->set_u_ub(tau_ub_);
      int_model_terminal->set_u_lb(tau_lb_);
      int_model_terminal->set_u_ub(tau_ub_);

      std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running(
          wp->knots - 1, diff_model_running);
      diff_models_running.push_back(diff_model_terminal);
      diff_models_running_.insert(diff_models_running_.end(), diff_models_running.begin(), diff_models_running.end());

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running(wp->knots - 1,
                                                                                        int_model_running);
      int_models_running.push_back(int_model_terminal);
      int_models_running_.insert(int_models_running_.end(), int_models_running.begin(), int_models_running.end());
    } else {
      int_model_running->set_u_lb(tau_lb_);
      int_model_running->set_u_ub(tau_ub_);

      std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running(
          wp->knots, diff_model_running);
      diff_models_running_.insert(diff_models_running_.end(), diff_models_running.begin(), diff_models_running.end());

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running(wp->knots, int_model_running);
      int_models_running_.insert(int_models_running_.end(), int_models_running.begin(), int_models_running.end());
      int_model_terminal_ = int_model_terminal;
    }
  }
  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(mission_->x0_, int_models_running_, int_model_terminal_);
  setSolver(solver_type);

  state_trajectory_ = std::vector<Eigen::VectorXd>(n_knots_, state_->zero());
}

boost::shared_ptr<crocoddyl::CostModelAbstract> TrajectoryGenerator::createCostStateRegularization() {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3).fill(1.);                        // Position 1.
  state_weights.segment(3, 3).fill(1.);                  // Orientation 1.
  state_weights.segment(model_->nv, 3).fill(1.);         // Linear velocity 1.
  state_weights.segment(model_->nv + 3, 3).fill(1000.);  // Angular velocity 1000.

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

void TrajectoryGenerator::solve() {
  problem_->set_x0(mission_->x0_);
  solver_->solve();
  // in the unit test check that the solve trajecotry and the stae_trajectory have the same size
  std::copy(solver_->get_xs().begin(), solver_->get_xs().end(), state_trajectory_.begin());
  state_hover_ = state_->zero();
  state_hover_.head(7) = solver_->get_xs().back().head(7);
}

const boost::shared_ptr<const Mission> TrajectoryGenerator::getMission() const { return mission_; }

std::vector<Eigen::VectorXd> TrajectoryGenerator::getTrajectoryPortion(const std::size_t& idx_init,
                                                                       const std::size_t& idx_end) const {
  assert(idx_init < idx_end);
  std::vector<Eigen::VectorXd>::const_iterator first = state_trajectory_.begin() + idx_init;
  std::vector<Eigen::VectorXd>::const_iterator last = state_trajectory_.begin() + idx_end + 1;
  return std::vector<Eigen::VectorXd>(first, last);
}

const Eigen::VectorXd& TrajectoryGenerator::getTrajectoryState(const std::size_t& cursor) const {
  if (cursor < state_trajectory_.size()) {
    return state_trajectory_[cursor];
  } else {
    return state_hover_;
  }
}

}  // namespace multicopter_mpc