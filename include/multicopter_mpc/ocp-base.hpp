#ifndef MULTICOPTER_MPC_OCP_BASE_HPP_
#define MULTICOPTER_MPC_OCP_BASE_HPP_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "pinocchio/multibody/model.hpp"

#include "crocoddyl/core/activations/weighted-quadratic-barrier.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

class OcpAbstract {
 public:
  OcpAbstract(const boost::shared_ptr<pinocchio::Model> model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const size_t& frame_base_link_id);
  ~OcpAbstract();

  virtual void createProblem() = 0;
  virtual boost::shared_ptr<crocoddyl::CostModelAbstract>& setCostStateRegularization();
  virtual boost::shared_ptr<crocoddyl::CostModelAbstract>& setCostControlRegularization();

 protected:
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<crocoddyl::StateMultibody> state_;

  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;

  Eigen::VectorXd state_weights_;  // Eventually check if this should be removed

  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> diff_models_running;
  std::vector<boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>> int_models_running;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_terminal;

  boost::shared_ptr<crocoddyl::ShootingProblem> problem_;

  size_t frame_base_link_id_;
  size_t knots_;
  double dt_;
};

}  // namespace multicopter_mpc

#endif  // MULTICOPTER_MPC_OCP_BASE_HPP_