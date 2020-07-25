#include "multicopter_mpc/ocp/ocp-base.hpp"

namespace multicopter_mpc {

OcpAbstract::OcpAbstract(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt)
    : model_(model), mc_params_(mc_params), dt_(dt) {
  state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);
  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state_, mc_params_->n_rotors_, mc_params_->tau_f_);

  tau_lb_ = Eigen::VectorXd(actuation_->get_nu());
  tau_ub_ = Eigen::VectorXd(actuation_->get_nu());
  tau_lb_.head(mc_params_->n_rotors_).fill(mc_params_->min_thrust_);
  tau_ub_.head(mc_params_->n_rotors_).fill(mc_params_->max_thrust_);

  state_initial_ = state_->zero();
  frame_base_link_id_ = model_->getFrameId(mc_params_->base_link_name_);
}

OcpAbstract::~OcpAbstract() {}

void OcpAbstract::initializeDefaultParameters(){};

void OcpAbstract::setSolver(const SolverTypes::Type& solver_type) {
  assert(problem_ != nullptr);

  switch (solver_type) {
    case SolverTypes::BoxFDDP: {
      std::cout << "Solver set: BoxFDDP" << std::endl;
      solver_ = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem_);
      break;
    }
    case SolverTypes::SquashBoxFDDP: {
      break;
    }
    default:
      break;
  }
}

void OcpAbstract::setSolverCallbacks(const bool& activated) {
  if (activated) {
    if (solver_callbacks_.size() == 0) {
      solver_callbacks_.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
    }
  } else {
    solver_callbacks_.clear();
  }
  solver_->setCallbacks(solver_callbacks_);
}

void OcpAbstract::setSolverIters(const std::size_t& n_iters) { solver_iters_ = n_iters; }

void OcpAbstract::solve() { solver_->solve(); }

const boost::shared_ptr<pinocchio::Model> OcpAbstract::getModel() const { return model_; }
const boost::shared_ptr<MultiCopterBaseParams> OcpAbstract::getMcParams() const { return mc_params_; }
const boost::shared_ptr<crocoddyl::StateMultibody> OcpAbstract::getStateMultibody() const { return state_; }
const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> OcpAbstract::getActuation() const {
  return actuation_;
}
const boost::shared_ptr<crocoddyl::ShootingProblem> OcpAbstract::getProblem() const { return problem_; }
const boost::shared_ptr<crocoddyl::SolverAbstract> OcpAbstract::getSolver() const { return solver_; }
const double& OcpAbstract::getTimeStep() const { return dt_; }
const Eigen::VectorXd& OcpAbstract::getActuationLowerBounds() const { return tau_lb_; }
const Eigen::VectorXd& OcpAbstract::getActuationUpperBounds() const { return tau_ub_; }
const Eigen::VectorXd& OcpAbstract::getInitialState() const { return state_initial_; }
const int& OcpAbstract::getBaseLinkId() const { return frame_base_link_id_; }
const std::size_t& OcpAbstract::getKnots() const {
  if (n_knots_ == 0) {
    std::cout << "WARN: number of knots is 0" << std::endl;
  }
  return n_knots_;
}
const std::string& OcpAbstract::getParametersPath() const { return parameters_yaml_path_; }

void OcpAbstract::setInitialState(const Eigen::VectorXd& initial_state) {
  assert(initial_state.size() == state_->get_nx());  // Might not be efficient to do this here
  state_initial_ = initial_state;
}

}  // namespace multicopter_mpc