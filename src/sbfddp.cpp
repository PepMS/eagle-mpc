#include "multicopter_mpc/sbfddp.hpp"

namespace multicopter_mpc {

SolverSbFDDP::SolverSbFDDP(boost::shared_ptr<crocoddyl::ShootingProblem> problem,
                           boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squashing_model)
    : crocoddyl::SolverFDDP(problem),
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

  barrier_quad_weights_aux_ = smooth_ * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
  barrier_quad_weights_ = 1. / barrier_quad_weights_aux_.array().pow(2);
  barrierInit();

  th_stop_gaps_ = 1e0;
  set_stoppingCriteria(crocoddyl::SolverDDP::StopCriteriaCostReduction);
  set_stoppingTest(crocoddyl::SolverFDDP::StoppingTestType::StopTestGaps);
  ddp_ = boost::make_shared<crocoddyl::SolverDDP>(problem_);
  ddp_->set_stoppingCriteria(crocoddyl::SolverDDP::StopCriteriaCostReduction);

  const std::size_t& T = problem_->get_T();
  us_squash_.resize(T);
  for (std::size_t t = 0; t < T; ++t) {
    us_squash_[t] = Eigen::VectorXd::Zero(squashing_model_->get_ns());
  }
}
SolverSbFDDP::~SolverSbFDDP() {}

void SolverSbFDDP::barrierInit() {
  barrier_act_bounds_ =
      boost::make_shared<crocoddyl::ActivationBounds>(squashing_model_->get_s_lb(), squashing_model_->get_s_ub(), 1.0);
  barrier_activation_ = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(*(barrier_act_bounds_),
                                                                                               barrier_quad_weights_);
  squash_barr_cost_ = boost::make_shared<crocoddyl::CostModelControl>(problem_->get_terminalModel()->get_state(),
                                                                      barrier_activation_, squashing_model_->get_ns());

  for (std::size_t i = 0; i < problem_->get_runningModels().size(); ++i) {
    euler_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem_->get_runningModels()[i]);
    if (euler_ != nullptr) {
      differential_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler_->get_differential());
    } else {
      rk4_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelRK4>(problem_->get_runningModels()[i]);
      if (rk4_ == nullptr) {
        MMPC_ERROR << "RK4 model is nullptr inside Squashing solver!";
      }
      differential_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(rk4_->get_differential());
    }

    if (differential_ == nullptr) {
      MMPC_ERROR << "Differential is nullptr inside Squashing solver!";
    }
    auto cost = differential_->get_costs()->get_costs().find("barrier");
    if (cost == differential_->get_costs()->get_costs().end()) {
      MMPC_INFO << "Added control barrier cost!";
      differential_->get_costs()->addCost("barrier", squash_barr_cost_, barrier_weight_);
    }
    problem_->updateModel(i, problem_->get_runningModels()[i]);
  }
}

bool SolverSbFDDP::solve(const std::vector<Eigen::VectorXd>& init_xs, const std::vector<Eigen::VectorXd>& init_us,
                         const std::size_t& maxiter, const bool& is_feasible, const double& regInit) {
  std::copy(init_xs.begin(), init_xs.end(), xs_.begin());
  std::copy(init_us.begin(), init_us.end(), us_.begin());

  smooth_ = smooth_init_;
  convergence_ = convergence_init_;

  total_iters_ = 0;
  while (convergence_ >= convergence_stop_) {
    squashingUpdate();
    barrierUpdate();

    th_stop_ = convergence_;
    crocoddyl::SolverFDDP::solve(xs_, us_, maxiter, false, reg_init_);

    smooth_ *= smooth_mult_;
    convergence_ *= convergence_mult_;
    total_iters_ += iter_ + 1;
  }

  if (!is_feasible_) {
    ddp_->set_th_stop(th_stop_);
    ddp_->setCallbacks(callbacks_);
    ddp_->solve(xs_, us_, maxiter, false, reg_init_);
    
    std::copy(ddp_->get_xs().begin(), ddp_->get_xs().end(), xs_.begin());
    std::copy(ddp_->get_us().begin(), ddp_->get_us().end(), us_.begin());
    total_iters_ += ddp_->get_iter() + 1;
  }

  iter_ = total_iters_ - 1;
  for (std::size_t i = 0; i < problem_->get_T(); ++i) {
    euler_d_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionDataEuler>(problem_->get_runningDatas()[i]);
    if (euler_d_ != nullptr) {
      differential_d_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionDataFreeFwdDynamics>(euler_d_->differential);
    } else {
      rk4_d_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionDataRK4>(problem_->get_runningDatas()[i]);
      if (rk4_d_ == nullptr) {
        MMPC_ERROR << "RK4 data is nullptr inside Squashing solver!";
      }
      differential_d_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionDataFreeFwdDynamics>(rk4_d_->differential[0]);
    }

    if (differential_d_ == nullptr) {
      MMPC_ERROR << "Differential Data is nullptr inside Squashing solver!";
    }
    actuation_squashing_d_ =
        boost::dynamic_pointer_cast<crocoddyl::ActuationSquashingData>(differential_d_->multibody.actuation);
    us_squash_[i] = actuation_squashing_d_->squashing->u;
  }

  return true;
}

void SolverSbFDDP::squashingUpdate() { squashing_model_->set_smooth(smooth_); }

void SolverSbFDDP::barrierUpdate() {
  barrier_quad_weights_aux_ = smooth_ * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
  barrier_quad_weights_ = 1. / barrier_quad_weights_aux_.array().pow(2);

  for (std::size_t i = 0; i < problem_->get_runningModels().size(); ++i) {
    euler_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem_->get_runningModels()[i]);
    if (euler_ != nullptr) {
      differential_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler_->get_differential());
    } else {
      rk4_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelRK4>(problem_->get_runningModels()[i]);
      if (rk4_ == nullptr) {
        MMPC_ERROR << "RK4 model is nullptr inside Squashing solver!";
      }
      differential_ =
          boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(rk4_->get_differential());
    }

    if (differential_ == nullptr) {
      MMPC_ERROR << "Differential is nullptr inside Squashing solver!";
    }
    barrier_activation_ = boost::dynamic_pointer_cast<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
        differential_->get_costs()->get_costs().find("barrier")->second->cost->get_activation());
    barrier_activation_->set_weights(barrier_quad_weights_);
    differential_->get_costs()->get_costs().find("barrier")->second->weight = barrier_weight_;
  }
}

const std::vector<Eigen::VectorXd>& SolverSbFDDP::getSquashControls() const { return us_squash_; }

}  // namespace multicopter_mpc