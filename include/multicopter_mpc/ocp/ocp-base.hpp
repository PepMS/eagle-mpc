#ifndef MULTICOPTER_MPC_OCP_BASE_HPP_
#define MULTICOPTER_MPC_OCP_BASE_HPP_

#include <cassert>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "pinocchio/multibody/model.hpp"

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/integrator/rk4.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solver-base.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/box-ddp.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

struct SolverTypes {
  enum Type { BoxFDDP, BoxDDP, SquashBoxFDDP, NbSolverTypes };
};

struct IntegratorTypes {
  enum Type { Euler, RK4, NbIntegratorTypes };
};

class OcpAbstract {
 public:
  // Constructor & Destructor
  OcpAbstract(const boost::shared_ptr<pinocchio::Model>& model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params);
  virtual ~OcpAbstract();

  // Pure virtual
  virtual void loadParameters(const std::string& yaml_path) = 0;
  /**
   * @brief Set time step for the optimal control problem.
   *
   * Pure virtual. Each class requires different problem modification depending on its specific purpose.
   *
   * @param[in]  dt     time step
   */
  virtual void setTimeStep(const double& dt) = 0;

  // Other methods
  virtual void solve(const std::vector<Eigen::VectorXd>& state_trajectory = crocoddyl::DEFAULT_VECTOR,
                     const std::vector<Eigen::VectorXd>& control_trajectory = crocoddyl::DEFAULT_VECTOR);
  void createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type,
                     const double& dt);

  // Setters
  void setSolverCallbacks(const bool& activated);
  void setInitialState(const Eigen::VectorXd& initial_state);
  void setSolverIters(const std::size_t& n_iters);
  void setSolverStopTh(const double& stop_th);
  void setIntegratorType(const IntegratorTypes::Type& integrator_type);

  // Getters
  boost::shared_ptr<pinocchio::Model> getModel() const;
  const boost::shared_ptr<MultiCopterBaseParams> getMcParams() const;
  const boost::shared_ptr<crocoddyl::StateMultibody> getStateMultibody() const;
  const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> getActuation() const;
  const boost::shared_ptr<crocoddyl::ShootingProblem> getProblem() const;
  const boost::shared_ptr<crocoddyl::SolverDDP> getSolver() const;
  const double& getTimeStep() const;
  const Eigen::VectorXd& getActuationLowerBounds() const;
  const Eigen::VectorXd& getActuationUpperBounds() const;
  const Eigen::VectorXd& getInitialState() const;
  const int& getBaseLinkId() const;
  const std::size_t& getKnots() const;
  const IntegratorTypes::Type& getIntegratorType() const;

 protected:
  // Methods
  virtual void createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type) = 0;
  virtual void initializeDefaultParameters();

  void setSolver(const SolverTypes::Type& solver_type);

  // Class members
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running_;
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running_;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal_;
  boost::shared_ptr<crocoddyl::ActionModelAbstract> int_model_terminal_;
  boost::shared_ptr<crocoddyl::ShootingProblem> problem_;

  boost::shared_ptr<crocoddyl::SolverDDP> solver_;
  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> solver_callbacks_;
  std::size_t solver_iters_;
  SolverTypes::Type solver_type_;
  IntegratorTypes::Type integrator_type_;

  int frame_base_link_id_;
  std::size_t n_knots_;
  double dt_;
  Eigen::VectorXd tau_ub_;
  Eigen::VectorXd tau_lb_;

  Eigen::VectorXd state_initial_;
};

}  // namespace multicopter_mpc

#endif  // MULTICOPTER_MPC_OCP_BASE_HPP_