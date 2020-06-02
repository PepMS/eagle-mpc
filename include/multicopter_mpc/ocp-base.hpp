#ifndef MULTICOPTER_MPC_OCP_BASE_HPP_
#define MULTICOPTER_MPC_OCP_BASE_HPP_

#include <cassert>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "pinocchio/multibody/model.hpp"

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solver-base.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {

class OcpAbstract {
 public:
  OcpAbstract(const boost::shared_ptr<pinocchio::Model> model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt);
  ~OcpAbstract();

  virtual void createProblem(const SolverTypes::Type& solver_type) = 0;

  virtual void setSolver(const SolverTypes::Type& solver_type);
  virtual void setSolverCallbacks(const bool& activated);
  virtual void solve();

  const boost::shared_ptr<const pinocchio::Model> getModel() const;
  const boost::shared_ptr<const MultiCopterBaseParams> getMcParams() const;
  const boost::shared_ptr<const crocoddyl::StateMultibody> getState() const;
  const boost::shared_ptr<const crocoddyl::ActuationModelMultiCopterBase> getActuation() const;
  const double& getTimeStep() const;
  const Eigen::VectorXd& getActuationLowerBounds() const;
  const Eigen::VectorXd& getActuationUpperBounds() const;
  const int& getBaseLinkId() const;



 protected:
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running_;
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running_;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal_;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_terminal_;
  boost::shared_ptr<crocoddyl::ShootingProblem> problem_;

  boost::shared_ptr<crocoddyl::SolverAbstract> solver_;
  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> solver_callbacks_;

  int frame_base_link_id_;
  std::size_t n_knots_;
  double dt_;
  Eigen::VectorXd tau_ub_;
  Eigen::VectorXd tau_lb_;
};

}  // namespace multicopter_mpc

#endif  // MULTICOPTER_MPC_OCP_BASE_HPP_