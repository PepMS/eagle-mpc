#include "multicopter_mpc/ocp/low-level-controller.hpp"

namespace multicopter_mpc {

LowLevelController::LowLevelController(const boost::shared_ptr<pinocchio::Model> model,
                                       const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                       std::size_t& n_knots)
    : OcpAbstract(model, mc_params, dt) {
  n_knots_ = n_knots;

  state_ref_.resize(n_knots_);
  for (std::size_t t = 0; t < n_knots_; ++t) {
    state_ref_[t] = state_->zero();
  }
}

LowLevelController::~LowLevelController() {}

void LowLevelController::createProblem(const SolverTypes::Type& solver_type) {
  for (int i = 0; i < n_knots_ - 1; ++i) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = createDifferentialModel(i);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

    diff_models_running_.push_back(diff_model);
    int_models_running_.push_back(int_model);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      createDifferentialModel(n_knots_ - 1);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

  diff_model_terminal_ = diff_model;
  int_model_terminal_ = int_model;

  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(state_initial_, int_models_running_, int_model_terminal_);
  setSolver(solver_type);
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> LowLevelController::createDifferentialModel(
    const unsigned int& trajectory_idx) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_state = createCostState(trajectory_idx);
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  // Regularitzations
  cost_model->addCost("x_cost", cost_state, 1e-2);
  cost_model->addCost("u_reg", cost_reg_control, 1e-4);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> LowLevelController::createCostState(
    const unsigned int& trajectory_idx) {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3).fill(1.);                     // Position 1.
  state_weights.segment(3, 3).fill(1.);               // Orientation 1.
  state_weights.segment(model_->nv, 3).fill(1.);      // Linear velocity 1.
  state_weights.segment(model_->nv + 3, 3).fill(1.);  // Angular velocity 1.

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = boost::make_shared<crocoddyl::CostModelState>(
      state_, activation_state, state_ref_[trajectory_idx], actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> LowLevelController::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

void LowLevelController::setReferenceStateTrajectory(const std::vector<Eigen::VectorXd>& state_trajectory) {
  state_ref_.clear();
  std::copy(state_trajectory.begin(), state_trajectory.end(), std::back_inserter(state_ref_));
}

void LowLevelController::updateReferenceStateTrajectory(const Eigen::Ref<Eigen::VectorXd>& state_new) {
  assert(state_new.size() == state_->get_nx());

  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    state_ref_[t] = state_ref_[t + 1];
  }
  state_ref_[n_knots_ - 1] = state_new;
}

void LowLevelController::solve() {
  // solver_->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR, solver_iters_, false, 1e-9);
  solver_->solve();
}

const Eigen::VectorXd& LowLevelController::getControls(const std::size_t& idx) const{
  std::cout << "This is the index: " << idx << std::endl;
  return solver_->get_us()[idx];
}


}  // namespace multicopter_mpc