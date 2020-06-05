#include "multicopter_mpc/ocp/low-level-controller.hpp"

namespace multicopter_mpc {

LowLevelController::LowLevelController(const boost::shared_ptr<pinocchio::Model>& model,
                                       const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                       std::size_t& n_knots)
    : OcpAbstract(model, mc_params, dt) {
  n_knots_ = n_knots;

  state_ref_.resize(n_knots_, state_->zero());

  initializeDefaultParameters();
}

LowLevelController::~LowLevelController() {}

void LowLevelController::initializeDefaultParameters() {
  params_.w_state = 1e-2;
  params_.w_state_position.fill(1.0);
  params_.w_state_orientation.fill(1.0);
  params_.w_state_velocity_lin.fill(1.0);
  params_.w_state_velocity_ang.fill(1.0);
  params_.w_control = 1e-4;
}

void LowLevelController::loadParameters(const yaml_parser::ParamsServer& server) {
  std::vector<std::string> state_weights = server.getParam<std::vector<std::string>>("ocp/state_weights");
  std::map<std::string, std::string> current_state =
      yaml_parser::converter<std::map<std::string, std::string>>::convert(state_weights[0]);

  params_.w_state_position = yaml_parser::converter<Eigen::Vector3d>::convert(current_state["position"]);
  params_.w_state_orientation = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["orientation"]);
  params_.w_state_velocity_lin = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_lin"]);
  params_.w_state_velocity_ang = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_ang"]);

  params_.w_state = server.getParam<double>("ocp/cost_state_weight");
  params_.w_control = server.getParam<double>("ocp/cost_control_weight");
}

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
  cost_model->addCost("state_error", cost_state, params_.w_state);
  cost_model->addCost("control", cost_reg_control, params_.w_control);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);
  
  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> LowLevelController::createCostState(
    const unsigned int& trajectory_idx) {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3) = params_.w_state_position;
  state_weights.segment(3, 3) = params_.w_state_orientation;
  state_weights.segment(model_->nv, 3) = params_.w_state_velocity_lin;
  state_weights.segment(model_->nv + 3, 3) = params_.w_state_velocity_lin;

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

void LowLevelController::updateReferenceStateTrajectory(const Eigen::Ref<Eigen::VectorXd>& state_new) {
  assert(state_new.size() == state_->get_nx());

  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    state_ref_[t] = state_ref_[t + 1];

    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        diff_models_running_[t]->get_costs()->get_costs().find("state_error")->second->cost);
    cost_state->set_xref(state_ref_[t]);
  }
  state_ref_[n_knots_ - 1] = state_new;
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      diff_model_terminal_->get_costs()->get_costs().find("state_error")->second->cost);
  cost_state->set_xref(state_ref_[n_knots_ - 1]);
}

void LowLevelController::solve() {
  problem_->set_x0(state_initial_);
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iters_, false, 1e-9);
}

const Eigen::VectorXd& LowLevelController::getControls(const std::size_t& idx) const { return solver_->get_us()[idx]; }
const std::vector<Eigen::VectorXd>& LowLevelController::getStateRef() const { return state_ref_; }
const LowLevelControllerParams& LowLevelController::getParams() const { return params_; }

void LowLevelController::setReferenceStateTrajectory(const std::vector<Eigen::VectorXd>& state_trajectory) {
  assert(state_ref_.size() == state_trajectory.size());
  std::copy(state_trajectory.begin(), state_trajectory.end(), state_ref_.begin());

  if (problem_ != nullptr) {
    for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
      boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
          diff_models_running_[t]->get_costs()->get_costs().find("state_error")->second->cost);
      cost_state->set_xref(state_ref_[t]);
    }
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        diff_model_terminal_->get_costs()->get_costs().find("state_error")->second->cost);
    cost_state->set_xref(state_ref_[n_knots_ - 1]);
  }
}

void LowLevelController::printCosts() {
  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        diff_models_running_[t]->get_costs()->get_costs().find("state_error")->second->cost);
    std::cout << "Node number " << t << ": \n" << cost_state->get_xref() << std::endl;
  }
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      diff_model_terminal_->get_costs()->get_costs().find("state_error")->second->cost);
  std::cout << "Node number " << n_knots_ - 1 << ": \n" << cost_state->get_xref() << std::endl;

}
}  // namespace multicopter_mpc