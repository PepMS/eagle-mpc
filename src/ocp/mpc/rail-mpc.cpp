#include "multicopter_mpc/ocp/mpc/rail-mpc.hpp"

namespace multicopter_mpc {

RailMpc::RailMpc(const boost::shared_ptr<pinocchio::Model>& model,
                                       const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                       const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots)
    : MpcAbstract(model, mc_params, dt, mission, n_knots) {
  
  initializeDefaultParameters();
}

RailMpc::~RailMpc() {}

std::string RailMpc::getFactoryName() { return "RailMpc"; }

boost::shared_ptr<MpcAbstract> RailMpc::createMpcController(
    const boost::shared_ptr<pinocchio::Model>& model, const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
    const double& dt, const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots) {
  return boost::make_shared<RailMpc>(model, mc_params, dt, mission, n_knots);
}

bool RailMpc::registered_ =
    FactoryMpc::registerMpcController(RailMpc::getFactoryName(), RailMpc::createMpcController);

void RailMpc::initializeDefaultParameters() {
  params_.w_state = 1e-2;
  params_.w_state_position.fill(1.0);
  params_.w_state_orientation.fill(1.0);
  params_.w_state_velocity_lin.fill(1.0);
  params_.w_state_velocity_ang.fill(1.0);
  params_.w_control = 1e-4;
}

void RailMpc::loadParameters(const yaml_parser::ParamsServer& server) {
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

void RailMpc::initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) {
  state_ref_.resize(n_knots_, state_->zero());
  control_ref_.resize(n_knots_ - 1, Eigen::VectorXd::Zero(4));

  trajectory_generator_->createProblem(solver_type);
  trajectory_generator_->solve();

  mission_ = trajectory_generator_->getMission();
}

void RailMpc::createProblem(const SolverTypes::Type& solver_type) {
  initializeTrajectoryGenerator(solver_type);
  
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

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> RailMpc::createDifferentialModel(
    const unsigned int& trajectory_idx) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_state = createCostState(trajectory_idx);
  if (trajectory_idx < n_knots_ - 1) {
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_control = createCostControl(trajectory_idx);
    cost_model->addCost("control_error", cost_control, params_.w_control);
  }

  // Regularitzations
  cost_model->addCost("state_error", cost_state, params_.w_state);

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> RailMpc::createCostState(
    const unsigned int& trajectory_idx) {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3) = params_.w_state_position;
  state_weights.segment(3, 3) = params_.w_state_orientation;
  state_weights.segment(model_->nv, 3) = params_.w_state_velocity_lin;
  state_weights.segment(model_->nv + 3, 3) = params_.w_state_velocity_ang;

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = boost::make_shared<crocoddyl::CostModelState>(
      state_, activation_state, state_ref_[trajectory_idx], actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> RailMpc::createCostControl(
    const unsigned int& trajectory_idx) {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, control_ref_[trajectory_idx]);

  return cost_control;
}

void RailMpc::solve() {
  problem_->set_x0(state_initial_);
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iters_, false, 1e-9);
}

void RailMpc::updateReferences(const Eigen::Ref<Eigen::VectorXd>& state_new,
                                          const Eigen::Ref<Eigen::VectorXd>& control_new) {
  assert(state_new.size() == state_->get_nx());

  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    state_ref_[t] = state_ref_[t + 1];

    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        diff_models_running_[t]->get_costs()->get_costs().find("state_error")->second->cost);
    cost_state->set_xref(state_ref_[t]);

    boost::shared_ptr<crocoddyl::CostModelControl> cost_control =
        boost::static_pointer_cast<crocoddyl::CostModelControl>(
            diff_models_running_[t]->get_costs()->get_costs().find("control_error")->second->cost);
    if (t < n_knots_ - 2) {
      control_ref_[t] = control_ref_[t + 1];
      cost_control->set_uref(control_ref_[t]);
    } else {
      control_ref_[t] = control_new;
      cost_control->set_uref(control_ref_[t]);
    }
  }

  state_ref_[n_knots_ - 1] = state_new;
  boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
      diff_model_terminal_->get_costs()->get_costs().find("state_error")->second->cost);
  cost_state->set_xref(state_ref_[n_knots_ - 1]);
}

void RailMpc::updateProblem(const std::size_t idx_trajectory) {}

const std::vector<Eigen::VectorXd>& RailMpc::getStateRef() const { return state_ref_; }
const RailMpcParams& RailMpc::getParams() const { return params_; }

void RailMpc::setReferences(const std::vector<Eigen::VectorXd>& state_trajectory,
                                       const std::vector<Eigen::VectorXd>& control_trajectory) {
  assert(state_ref_.size() == state_trajectory.size());
  std::copy(state_trajectory.begin(), state_trajectory.end(), state_ref_.begin());
  std::copy(control_trajectory.begin(), control_trajectory.end(), control_ref_.begin());

  if (problem_ != nullptr) {
    for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
      boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
          diff_models_running_[t]->get_costs()->get_costs().find("state_error")->second->cost);
      cost_state->set_xref(state_ref_[t]);
      boost::shared_ptr<crocoddyl::CostModelControl> cost_control =
          boost::static_pointer_cast<crocoddyl::CostModelControl>(
              diff_models_running_[t]->get_costs()->get_costs().find("control_error")->second->cost);
      cost_control->set_uref(control_ref_[t]);
    }
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        diff_model_terminal_->get_costs()->get_costs().find("state_error")->second->cost);
    cost_state->set_xref(state_ref_[n_knots_ - 1]);
  }
}

void RailMpc::printCosts() {
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