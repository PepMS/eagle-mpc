#ifndef MULTICOPTER_MPC_SBFDDP_HPP_
#define MULTICOPTER_MPC_SBFDDP_HPP_

#include <Eigen/Dense>

#include "crocoddyl/core/solver-base.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic-barrier.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/integrator/rk4.hpp"

#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {
class SolverSbFDDP {
 public:
  SolverSbFDDP(boost::shared_ptr<crocoddyl::ShootingProblem> problem,
               boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squashing_model);
  ~SolverSbFDDP();

  bool solve(const std::vector<Eigen::VectorXd>& init_xs = crocoddyl::DEFAULT_VECTOR,
             const std::vector<Eigen::VectorXd>& init_us = crocoddyl::DEFAULT_VECTOR,
             const std::size_t& maxiter = 100);

  void setCallbacks(const std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract> >& callbacks);

 private:
  void barrierInit();
  void squashingUpdate();
  void barrierUpdate();

  boost::shared_ptr<crocoddyl::ShootingProblem> problem_;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squashing_model_;
  boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation_;
  boost::shared_ptr<crocoddyl::SolverDDP> ddp_;
  boost::shared_ptr<crocoddyl::SolverFDDP> fddp_;
  boost::shared_ptr<crocoddyl::ActivationBounds> barrier_act_bounds_;
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuadraticBarrier> barrier_activation_;
  boost::shared_ptr<crocoddyl::CostModelControl> squash_barr_cost_;
  double smooth_;
  double smooth_init_;
  double smooth_mult_;

  Eigen::VectorXd barrier_quad_weights_;
  double barrier_weight_;

  double convergence_;
  double convergence_init_;
  double convergence_stop_;
  double convergence_mult_;

  std::size_t max_iters_;
  double reg_init_;

  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
};
}  // namespace multicopter_mpc

#endif