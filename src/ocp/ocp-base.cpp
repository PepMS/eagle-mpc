#include "multicopter_mpc/ocp/ocp-base.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

OcpAbstract::OcpAbstract(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params)
    : model_(model), mc_params_(mc_params) {
  state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);
  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state_, mc_params_->n_rotors_, mc_params_->tau_f_);

  tau_lb_ = Eigen::VectorXd(actuation_->get_nu());
  tau_ub_ = Eigen::VectorXd(actuation_->get_nu());
  tau_lb_.head(mc_params_->n_rotors_).fill(mc_params_->min_thrust_);
  tau_ub_.head(mc_params_->n_rotors_).fill(mc_params_->max_thrust_);

  state_initial_ = state_->zero();
  frame_base_link_id_ = model_->getFrameId(mc_params_->base_link_name_);

  solver_iters_ = 100;
  solver_type_ = SolverTypes::NbSolverTypes;
  integrator_type_ = IntegratorTypes::NbIntegratorTypes;
  n_knots_ = 100;
  dt_ = 0.0;
}

OcpAbstract::~OcpAbstract() {}

void OcpAbstract::initializeDefaultParameters(){};

void OcpAbstract::createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type,
                                const double& dt) {
  assert(dt > 0.0);
  assert(solver_type < SolverTypes::NbSolverTypes);
  assert(integrator_type < IntegratorTypes::NbIntegratorTypes);

  setTimeStep(dt);
  solver_type_ = solver_type;
  integrator_type_ = integrator_type;
  createProblem(solver_type, integrator_type);
}

void OcpAbstract::solve(const std::vector<Eigen::VectorXd>& state_trajectory,
                        const std::vector<Eigen::VectorXd>& control_trajectory) {
  solver_->solve(state_trajectory, control_trajectory, solver_iters_, false);
  std::cout << "Solve!" << std::endl;
}

void OcpAbstract::setSolver(const SolverTypes::Type& solver_type) {
  assert(problem_ != nullptr);

  switch (solver_type) {
    case SolverTypes::BoxFDDP: {
      MMPC_INFO << "Solver set: BoxFDDP";
      solver_ = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem_);
      break;
    }
    case SolverTypes::BoxDDP: {
      MMPC_INFO << "Solver set: BoxDDP";
      solver_ = boost::make_shared<crocoddyl::SolverBoxDDP>(problem_);
      break;
    }
    case SolverTypes::SquashBoxFDDP: {
      break;
    }
    default:
      MMPC_INFO << "Solver set: BoxFDDP";
      solver_ = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem_);
      break;
  }

  solver_type_ = solver_type;
}

void OcpAbstract::setSolverCallbacks(const bool& activated) {
  assert(solver_ != nullptr);

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

void OcpAbstract::setSolverStopTh(const double& stop_th) { solver_->set_th_stop(stop_th); }

void OcpAbstract::setInitialState(const Eigen::VectorXd& initial_state) {
  assert(initial_state.size() == state_->get_nx());  // Might not be efficient to do this here
  state_initial_ = initial_state;

  if (problem_ != nullptr) {
    problem_->set_x0(state_initial_);
  }
}

void OcpAbstract::setIntegratorType(const IntegratorTypes::Type& integrator_type) {
  if (integrator_type == integrator_type_) {
    return;
  } else {
    integrator_type_ = integrator_type;
    if (problem_ != nullptr) {
      int_models_running_.clear();
      int_model_terminal_ = nullptr;

      switch (integrator_type_) {
        case IntegratorTypes::Euler:
          for (std::size_t i = 0; i < diff_models_running_.size(); ++i) {
            int_models_running_.push_back(
                boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_models_running_[i], dt_));
          }
          int_model_terminal_ = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal_, 0.);
          break;
        case IntegratorTypes::RK4:
          for (std::size_t i = 0; i < diff_models_running_.size(); ++i) {
            int_models_running_.push_back(
                boost::make_shared<crocoddyl::IntegratedActionModelRK4>(diff_models_running_[i], dt_));
          }
          int_model_terminal_ = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(diff_model_terminal_, 0.);
          break;
        default:
          for (std::size_t i = 0; i < diff_models_running_.size(); ++i) {
            int_models_running_.push_back(
                boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_models_running_[i], dt_));
          }
          int_model_terminal_ = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal_, 0.);
          break;
      }
    }
  }
}

const boost::shared_ptr<pinocchio::Model> OcpAbstract::getModel() const { return model_; }
const boost::shared_ptr<MultiCopterBaseParams> OcpAbstract::getMcParams() const { return mc_params_; }
const boost::shared_ptr<crocoddyl::StateMultibody> OcpAbstract::getStateMultibody() const { return state_; }
const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> OcpAbstract::getActuation() const {
  return actuation_;
}
const boost::shared_ptr<crocoddyl::ShootingProblem> OcpAbstract::getProblem() const { return problem_; }
const boost::shared_ptr<crocoddyl::SolverDDP> OcpAbstract::getSolver() const { return solver_; }
const double& OcpAbstract::getTimeStep() const { return dt_; }
const Eigen::VectorXd& OcpAbstract::getActuationLowerBounds() const { return tau_lb_; }
const Eigen::VectorXd& OcpAbstract::getActuationUpperBounds() const { return tau_ub_; }
const Eigen::VectorXd& OcpAbstract::getInitialState() const { return state_initial_; }
const int& OcpAbstract::getBaseLinkId() const { return frame_base_link_id_; }
const std::size_t& OcpAbstract::getKnots() const {
  if (n_knots_ == 0) {
    MMPC_WARN << "number of knots is 0";
  }
  return n_knots_;
}
const IntegratorTypes::Type& OcpAbstract::getIntegratorType() const { return integrator_type_; }

}  // namespace multicopter_mpc