#include "sbfddp.hpp"

namespace multicopter_mpc {
SolverSbFDDP::SolverSbFDDP(boost::shared_ptr<crocoddyl::ShootingProblem> problem,
                           boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squashing_model)
    : problem_(problem),
      squashing_model_(squashing_model),
      smooth_init_(0.1),
      smooth_mult_(0.5),
      barrier_weight_(1e-3),
      convergence_init_(1e-2),
      convergence_stop_(1e-3),
      convergence_mult_(1e-1),
      max_iters_(100),
      reg_init_(1e-9) {
  
  smooth_ = smooth_init_;
  convergence_ = convergence_init_;

  Eigen::VectorXd aux = smooth_ * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
  barrier_quad_weights_ = 1. / aux.array().pow(2);
  barrierInit();

  fddp_ = boost::make_shared<crocoddyl::SolverFDDP>(problem_);
  fddp_->set_th_stop_gaps(1e0);
  fddp_->set_stoppingCriteria(crocoddyl::SolverDDP::StopCriteriaCostReduction);
  fddp_->set_stoppingTest(crocoddyl::SolverFDDP::StoppingTestType::StopTestGaps);
  ddp_ = boost::make_shared<crocoddyl::SolverDDP>(problem_);
  ddp_->set_stoppingCriteria(crocoddyl::SolverDDP::StopCriteriaCostReduction);

  const std::size_t& T = problem_->get_T();
  xs_.resize(T + 1);
  us_.resize(T);
  for (std::size_t t = 0; t < T; ++t) {
    if (t == 0) {
      xs_[t] = problem_->get_x0();
    } else {
      xs_[t] = problem_->get_terminalModel()->get_state()->zero();
    }
    us_[t] = Eigen::VectorXd::Zero(squashing_model_->get_ns());
  }
  xs_.back() = problem_->get_terminalModel()->get_state()->zero();

}
SolverSbFDDP::~SolverSbFDDP() {}

void SolverSbFDDP::barrierInit() {
  barrier_act_bounds_ =
      boost::make_shared<crocoddyl::ActivationBounds>(squashing_model_->get_s_lb(), squashing_model_->get_s_ub(), 1.0);
  barrier_activation_ = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(barrier_act_bounds_,
                                                                                               barrier_quad_weights_);
  squash_barr_cost_ = boost::make_shared<crocoddyl::CostModelControl>(problem_->get_terminalModel()->get_state(),
                                                                      barrier_activation_, squashing_model_->get_ns());

  for (std::size_t i = 0; i < problem_->get_runningModels().size(); ++i) {
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> euler =
        boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem_->get_runningModels()[i]);
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> differential;

    if (euler != nullptr) {
      differential =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler->get_differential());
    } else {
      boost::shared_ptr<crocoddyl::IntegratedActionModelRK4> rk4 =
          boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelRK4>(problem_->get_runningModels()[i]);
      if (rk4 == nullptr) {
        MMPC_ERROR << "RK4 model is nullptr inside Squashing solver!";
      }
      differential =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(rk4->get_differential());
    }

    if (differential == nullptr) {
      MMPC_ERROR << "Differential is nullptr inside Squashing solver!";
    }
    differential->get_costs()->addCost("barrier", squash_barr_cost_, barrier_weight_);
    problem_->updateModel(i, problem_->get_runningModels()[i]);
  }
}

void SolverSbFDDP::setCallbacks(const std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract> >& callbacks) {
  fddp_->setCallbacks(callbacks);
  ddp_->setCallbacks(callbacks);
}

bool SolverSbFDDP::solve(const std::vector<Eigen::VectorXd>& init_xs, const std::vector<Eigen::VectorXd>& init_us,
                         const std::size_t& maxiter) {
  std::copy(init_xs.begin(), init_xs.end(), xs_.begin());
  std::copy(init_us.begin(), init_us.end(), us_.begin());

  while (convergence_ >= convergence_stop_ ) {

  }
}

void SolverSbFDDP::squashingUpdate() {
 // assign variable actuation when initializing!
}

void barrierUpdate() {

}


}  // namespace multicopter_mpc