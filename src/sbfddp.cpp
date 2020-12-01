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
      reg_init_(1e-9),
      th_acceptnegstep_(2) {
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
      differential_->get_costs()->addCost("barrier", squash_barr_cost_, barrier_weight_);
    }
    problem_->updateModel(i, problem_->get_runningModels()[i]);
  }
}

bool SolverSbFDDP::solve(const std::vector<Eigen::VectorXd>& init_xs, const std::vector<Eigen::VectorXd>& init_us,
                         const std::size_t& maxiter, const bool& is_feasible, const double& reginit) {
  xs_try_[0] = problem_->get_x0();  // it is needed in case that init_xs[0] is infeasible
  setCandidate(init_xs, init_us, is_feasible);

  smooth_ = smooth_init_;
  convergence_ = convergence_init_;

  total_iters_ = 0;
  while (convergence_ >= convergence_stop_) {
    squashingUpdate();
    barrierUpdate();

    th_stop_ = convergence_;
    solveFDDP(maxiter, false, reg_init_);

    smooth_ *= smooth_mult_;
    convergence_ *= convergence_mult_;
    total_iters_ += iter_ + 1;
  }

  if (!is_feasible_) {
    solveDDP(maxiter, is_feasible_, reg_init_);
    total_iters_ += iter_ + 1;
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

bool SolverSbFDDP::solveFDDP(const std::size_t& maxiter, const bool& is_feasible, const double& reginit) {
  is_feasible_ = is_feasible;
  if (std::isnan(reginit)) {
    xreg_ = regmin_;
    ureg_ = regmin_;
  } else {
    xreg_ = reginit;
    ureg_ = reginit;
  }
  was_feasible_ = false;

  bool recalcDiff = true;
  for (iter_ = 0; iter_ < maxiter; ++iter_) {
    while (true) {
      try {
        computeDirection(recalcDiff);
      } catch (std::exception& e) {
        recalcDiff = false;
        increaseRegularization();
        if (xreg_ == regmax_) {
          return false;
        } else {
          continue;
        }
      }
      break;
    }
    updateExpectedImprovement();

    // We need to recalculate the derivatives when the step length passes
    recalcDiff = false;
    for (std::vector<double>::const_iterator it = alphas_.begin(); it != alphas_.end(); ++it) {
      steplength_ = *it;

      try {
        dV_ = tryStep(steplength_);
      } catch (std::exception& e) {
        continue;
      }
      expectedImprovement();
      dVexp_ = steplength_ * (d_[0] + 0.5 * steplength_ * d_[1]);

      if (dVexp_ >= 0) {  // descend direction
        if (d_[0] < th_grad_ || dV_ > th_acceptstep_ * dVexp_) {
          was_feasible_ = is_feasible_;
          setCandidate(xs_try_, us_try_, (was_feasible_) || (steplength_ == 1));
          cost_prev_ = cost_;
          cost_ = cost_try_;
          recalcDiff = true;
          break;
        }
      } else {  // reducing the gaps by allowing a small increment in the cost value
        if (dV_ > th_acceptnegstep_ * dVexp_) {
          was_feasible_ = is_feasible_;
          setCandidate(xs_try_, us_try_, (was_feasible_) || (steplength_ == 1));
          cost_prev_ = cost_;
          cost_ = cost_try_;
          recalcDiff = true;
          break;
        }
      }
    }

    if (steplength_ > th_stepdec_) {
      decreaseRegularization();
    }
    if (steplength_ <= th_stepinc_) {
      increaseRegularization();
      if (xreg_ == regmax_) {
        return false;
      }
    }
    stoppingCriteria();

    const std::size_t& n_callbacks = callbacks_.size();
    for (std::size_t c = 0; c < n_callbacks; ++c) {
      crocoddyl::CallbackAbstract& callback = *callbacks_[c];
      callback(*this);
    }

    if (stoppingTest()) {
      return true;
    }
  }
  return false;
}

bool SolverSbFDDP::solveDDP(const std::size_t& maxiter, const bool& is_feasible, const double& reginit) {
  xs_try_[0] = problem_->get_x0();  // it is needed in case that init_xs[0] is infeasible

  if (std::isnan(reginit)) {
    xreg_ = regmin_;
    ureg_ = regmin_;
  } else {
    xreg_ = reginit;
    ureg_ = reginit;
  }
  was_feasible_ = false;

  bool recalcDiff = true;
  for (iter_ = 0; iter_ < maxiter; ++iter_) {
    while (true) {
      try {
        computeDirection(recalcDiff);
      } catch (std::exception& e) {
        recalcDiff = false;
        increaseRegularization();
        if (xreg_ == regmax_) {
          return false;
        } else {
          continue;
        }
      }
      break;
    }
    expectedImprovementDDP();

    // We need to recalculate the derivatives when the step length passes
    recalcDiff = false;
    for (std::vector<double>::const_iterator it = alphas_.begin(); it != alphas_.end(); ++it) {
      steplength_ = *it;

      try {
        dV_ = tryStepDDP(steplength_);
      } catch (std::exception& e) {
        continue;
      }
      dVexp_ = steplength_ * (d_[0] + 0.5 * steplength_ * d_[1]);

      if (dVexp_ >= 0) {  // descend direction
        if (d_[0] < th_grad_ || !is_feasible_ || dV_ > th_acceptstep_ * dVexp_) {
          was_feasible_ = is_feasible_;
          setCandidate(xs_try_, us_try_, true);
          cost_prev_ = cost_;
          cost_ = cost_try_;
          recalcDiff = true;
          break;
        }
      }
    }

    if (steplength_ > th_stepdec_) {
      decreaseRegularization();
    }
    if (steplength_ <= th_stepinc_) {
      increaseRegularization();
      if (xreg_ == regmax_) {
        return false;
      }
    }
    stoppingCriteria();

    const std::size_t& n_callbacks = callbacks_.size();
    for (std::size_t c = 0; c < n_callbacks; ++c) {
      crocoddyl::CallbackAbstract& callback = *callbacks_[c];
      callback(*this);
    }

    if (stoppingTestFeasible()) {
      return true;
    }
  }
  return false;
}

const Eigen::Vector2d& SolverSbFDDP::expectedImprovementDDP() {
  d_.fill(0);
  const std::size_t& T = this->problem_->get_T();
  const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> >& models = problem_->get_runningModels();
  for (std::size_t t = 0; t < T; ++t) {
    const std::size_t& nu = models[t]->get_nu();
    if (nu != 0) {
      d_[0] += Qu_[t].head(nu).dot(k_[t].head(nu));
      d_[1] -= k_[t].head(nu).dot(Quuk_[t].head(nu));
    }
  }
  return d_;
}

double SolverSbFDDP::tryStepDDP(const double& steplength) {
  forwardPassDDP(steplength);
  return cost_ - cost_try_;
}

void SolverSbFDDP::forwardPassDDP(const double& steplength) {
  if (steplength > 1. || steplength < 0.) {
    throw_pretty("Invalid argument: "
                 << "invalid step length, value is between 0. to 1.");
  }
  cost_try_ = 0.;
  const std::size_t& T = problem_->get_T();
  const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> >& models = problem_->get_runningModels();
  const std::vector<boost::shared_ptr<crocoddyl::ActionDataAbstract> >& datas = problem_->get_runningDatas();
  for (std::size_t t = 0; t < T; ++t) {
    const boost::shared_ptr<crocoddyl::ActionModelAbstract>& m = models[t];
    const boost::shared_ptr<crocoddyl::ActionDataAbstract>& d = datas[t];

    m->get_state()->diff(xs_[t], xs_try_[t], dx_[t]);
    if (m->get_nu() != 0) {
      const std::size_t& nu = m->get_nu();

      us_try_[t].head(nu).noalias() = us_[t].head(nu);
      us_try_[t].head(nu).noalias() -= k_[t].head(nu) * steplength;
      us_try_[t].head(nu).noalias() -= K_[t].topRows(nu) * dx_[t];
      m->calc(d, xs_try_[t], us_try_[t].head(nu));
    } else {
      m->calc(d, xs_try_[t]);
    }
    xs_try_[t + 1] = d->xnext;
    cost_try_ += d->cost;

    if (crocoddyl::raiseIfNaN(cost_try_)) {
      throw_pretty("forward_error");
    }
    if (crocoddyl::raiseIfNaN(xs_try_[t + 1].lpNorm<Eigen::Infinity>())) {
      throw_pretty("forward_error");
    }
  }

  const boost::shared_ptr<crocoddyl::ActionModelAbstract>& m = problem_->get_terminalModel();
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& d = problem_->get_terminalData();
  m->calc(d, xs_try_.back());
  cost_try_ += d->cost;

  if (crocoddyl::raiseIfNaN(cost_try_)) {
    throw_pretty("forward_error");
  }
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