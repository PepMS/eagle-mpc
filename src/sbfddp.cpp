#include "eagle_mpc/sbfddp.hpp"

namespace eagle_mpc
{
SolverSbFDDP::SolverSbFDDP(boost::shared_ptr<crocoddyl::ShootingProblem>         problem,
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
      th_acceptnegstep_(2)
{
    smooth_      = smooth_init_;
    convergence_ = convergence_init_;

    barrier_quad_weights_aux_ =
        smooth_ * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
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

IntegratedActionModelTypes SolverSbFDDP::getIntegratedModelType(
    boost::shared_ptr<crocoddyl::ActionModelAbstract> int_action)
{
    euler_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelEuler>(int_action);
    if (euler_ != nullptr) {
        return IntegratedActionModelTypes::IntegratedActionModelEuler;
    } else {
        rk4_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionModelRK4>(int_action);
        if (rk4_ != nullptr) {
            return IntegratedActionModelTypes::IntegratedActionModelRK4;
        }
    }
    EMPC_ERROR("Integrated action type not found");
    return IntegratedActionModelTypes::NbIntegratedActionModelTypes;
}

DifferentialActionModelTypes SolverSbFDDP::getDifferentialModelType(
    boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> diff_action)
{
    free_ = boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(diff_action);
    if (free_ != nullptr) {
        return DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics;
    } else {
        contact_ = boost::dynamic_pointer_cast<crocoddyl::DifferentialActionModelContactFwdDynamics>(diff_action);
        if (contact_ != nullptr) {
            return DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics;
        }
    }
    EMPC_ERROR("Differential action type not found");
    return DifferentialActionModelTypes::NbDifferentialActionModelTypes;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> SolverSbFDDP::getDifferentialModelFromIntegrated(
    boost::shared_ptr<crocoddyl::ActionModelAbstract> int_action)
{
    switch (getIntegratedModelType(int_action)) {
        case IntegratedActionModelTypes::IntegratedActionModelEuler:
            return euler_->get_differential();
            break;
        case IntegratedActionModelTypes::IntegratedActionModelRK4:
            return rk4_->get_differential();
            break;
        default:
            return nullptr;
            break;
    }
}

boost::shared_ptr<crocoddyl::CostModelSum> SolverSbFDDP::getCostsFromDifferentialModel(
    boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> diff_action)
{
    switch (getDifferentialModelType(diff_action)) {
        case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics:
            return free_->get_costs();
            break;
        case DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics:
            return contact_->get_costs();
            break;
        default:
            return nullptr;
            break;
    }
}

IntegratedActionModelTypes SolverSbFDDP::getIntegratedDataType(
    boost::shared_ptr<crocoddyl::ActionDataAbstract> int_action)
{
    euler_d_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionDataEuler>(int_action);
    if (euler_d_ != nullptr) {
        return IntegratedActionModelTypes::IntegratedActionModelEuler;
    } else {
        rk4_d_ = boost::dynamic_pointer_cast<crocoddyl::IntegratedActionDataRK4>(int_action);
        if (rk4_d_ != nullptr) {
            return IntegratedActionModelTypes::IntegratedActionModelRK4;
        }
    }
    EMPC_ERROR("Integrated action data type not found");
    return IntegratedActionModelTypes::NbIntegratedActionModelTypes;
}

DifferentialActionModelTypes SolverSbFDDP::getDifferentialDataType(
    boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> diff_action)
{
    free_d_ = boost::dynamic_pointer_cast<crocoddyl::DifferentialActionDataFreeFwdDynamics>(diff_action);
    if (free_d_ != nullptr) {
        return DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics;
    } else {
        contact_d_ = boost::dynamic_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(diff_action);
        if (contact_d_ != nullptr) {
            return DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics;
        }
    }
    EMPC_ERROR("Differential action data type not found");
    return DifferentialActionModelTypes::NbDifferentialActionModelTypes;
}

boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> SolverSbFDDP::getDifferentialDataFromIntegrated(
    boost::shared_ptr<crocoddyl::ActionDataAbstract> int_action)
{
    switch (getIntegratedDataType(int_action)) {
        case IntegratedActionModelTypes::IntegratedActionModelEuler:
            return euler_d_->differential;
            break;
        case IntegratedActionModelTypes::IntegratedActionModelRK4:
            return rk4_d_->differential[0];
            break;
        default:
            return nullptr;
            break;
    }
}

boost::shared_ptr<crocoddyl::ActuationDataAbstract> SolverSbFDDP::getActuationDataFromDifferential(
    boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> diff_action)
{
    switch (getDifferentialDataType(diff_action)) {
        case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics:
            return free_d_->multibody.actuation;
            break;
        case DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics:
            return contact_d_->multibody.actuation;
            break;
        default:
            return nullptr;
            break;
    }
}

void SolverSbFDDP::barrierInit()
{
    barrier_act_bounds_ = boost::make_shared<crocoddyl::ActivationBounds>(squashing_model_->get_s_lb(),
                                                                          squashing_model_->get_s_ub(), 1.0);
    barrier_activation_ = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
        *(barrier_act_bounds_), barrier_quad_weights_);
    squash_barr_residual_ = boost::make_shared<crocoddyl::ResidualModelControl>(
        problem_->get_terminalModel()->get_state(), squashing_model_->get_ns());

    squash_barr_cost_ = boost::make_shared<crocoddyl::CostModelResidual>(problem_->get_terminalModel()->get_state(),
                                                                        barrier_activation_, squash_barr_residual_);

    for (std::size_t i = 0; i < problem_->get_runningModels().size(); ++i) {
        costs_ = getCostsFromDifferentialModel(getDifferentialModelFromIntegrated(problem_->get_runningModels()[i]));
        auto cost = costs_->get_costs().find("barrier");
        if (cost == costs_->get_costs().end()) {
            costs_->addCost("barrier", squash_barr_cost_, barrier_weight_);
        }
        problem_->updateModel(i, problem_->get_runningModels()[i]);
    }
    problem_->updateModel(problem_->get_T(), problem_->get_terminalModel());
}

bool SolverSbFDDP::solve(const std::vector<Eigen::VectorXd>& init_xs,
                         const std::vector<Eigen::VectorXd>& init_us,
                         const std::size_t                   maxiter,
                         const bool                          is_feasible,
                         const double                        regInit)
{
    xs_try_[0] = problem_->get_x0();  // it is needed in case that init_xs[0] is infeasible
    setCandidate(init_xs, init_us, is_feasible);

    smooth_      = smooth_init_;
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
    fillSquashedOutputs();

    return true;
}

bool SolverSbFDDP::solveFDDP(const std::size_t& maxiter, const bool& is_feasible, const double& reginit)
{
    is_feasible_ = is_feasible;
    if (std::isnan(reginit)) {
        xreg_ = reg_min_;
        ureg_ = reg_min_;
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
                if (xreg_ == reg_max_) {
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
                    cost_      = cost_try_;
                    recalcDiff = true;
                    break;
                }
            } else {  // reducing the gaps by allowing a small increment in the cost value
                if (dV_ > th_acceptnegstep_ * dVexp_) {
                    was_feasible_ = is_feasible_;
                    setCandidate(xs_try_, us_try_, (was_feasible_) || (steplength_ == 1));
                    cost_prev_ = cost_;
                    cost_      = cost_try_;
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
            if (xreg_ == reg_max_) {
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
    iter_ = iter_ >= maxiter ? maxiter - 1 : iter_;
    return false;
}

bool SolverSbFDDP::solveDDP(const std::size_t& maxiter, const bool& is_feasible, const double& reginit)
{
    if (std::isnan(reginit)) {
        xreg_ = reg_min_;
        ureg_ = reg_min_;
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
                if (xreg_ == reg_max_) {
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
                    cost_      = cost_try_;
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
            if (xreg_ == reg_max_) {
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
    iter_ = iter_ >= maxiter ? maxiter - 1 : iter_;
    return false;
}

const Eigen::Vector2d& SolverSbFDDP::expectedImprovementDDP()
{
    d_.fill(0);
    const std::size_t&                                                     T      = this->problem_->get_T();
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

double SolverSbFDDP::tryStepDDP(const double& steplength)
{
    forwardPassDDP(steplength);
    return cost_ - cost_try_;
}

void SolverSbFDDP::forwardPassDDP(const double& steplength)
{
    if (steplength > 1. || steplength < 0.) {
        throw_pretty("Invalid argument: "
                     << "invalid step length, value is between 0. to 1.");
    }
    cost_try_                                                                     = 0.;
    const std::size_t&                                                     T      = problem_->get_T();
    const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> >& models = problem_->get_runningModels();
    const std::vector<boost::shared_ptr<crocoddyl::ActionDataAbstract> >&  datas  = problem_->get_runningDatas();
    for (std::size_t t = 0; t < T; ++t) {
        const boost::shared_ptr<crocoddyl::ActionModelAbstract>& m = models[t];
        const boost::shared_ptr<crocoddyl::ActionDataAbstract>&  d = datas[t];

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
    const boost::shared_ptr<crocoddyl::ActionDataAbstract>&  d = problem_->get_terminalData();
    m->calc(d, xs_try_.back());
    cost_try_ += d->cost;

    if (crocoddyl::raiseIfNaN(cost_try_)) {
        throw_pretty("forward_error");
    }
}

void SolverSbFDDP::squashingUpdate() { squashing_model_->set_smooth(smooth_); }

void SolverSbFDDP::barrierUpdate()
{
    barrier_quad_weights_aux_ =
        smooth_ * (squashing_model_->get_s_ub().array() - squashing_model_->get_s_lb().array());
    barrier_quad_weights_ = 1. / barrier_quad_weights_aux_.array().pow(2);

    for (std::size_t i = 0; i < problem_->get_runningModels().size(); ++i) {
        costs_ = getCostsFromDifferentialModel(getDifferentialModelFromIntegrated(problem_->get_runningModels()[i]));
        barrier_activation_ = boost::dynamic_pointer_cast<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
            costs_->get_costs().find("barrier")->second->cost->get_activation());
        barrier_activation_->set_weights(barrier_quad_weights_);
        costs_->get_costs().find("barrier")->second->weight = barrier_weight_;
    }
}

void SolverSbFDDP::fillSquashedOutputs()
{
    for (std::size_t i = 0; i < problem_->get_T(); ++i) {
        actuation_squashing_d_ = boost::dynamic_pointer_cast<crocoddyl::ActuationSquashingData>(
            getActuationDataFromDifferential(getDifferentialDataFromIntegrated(problem_->get_runningDatas()[i])));
        us_squash_[i] = actuation_squashing_d_->squashing->u;
    }
}
const std::vector<Eigen::VectorXd>& SolverSbFDDP::getSquashControls() const { return us_squash_; }

const double& SolverSbFDDP::get_convergence_init() const { return convergence_init_; }

void SolverSbFDDP::set_convergence_init(const double& convergence_init) { convergence_init_ = convergence_init; }

}  // namespace eagle_mpc