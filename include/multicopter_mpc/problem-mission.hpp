#ifndef MULTICOPTER_MPC_PROBLEM_MISSION_HPP_
#define MULTICOPTER_MPC_PROBLEM_MISSION_HPP_

#include "pinocchio/multibody/model.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {
class ProblemMission {
 public:
  ProblemMission(boost::shared_ptr<Mission> mission, boost::shared_ptr<MultiCopterBaseParams> mc_params,
                 boost::shared_ptr<pinocchio::Model> mc_model,
                 boost::shared_ptr<crocoddyl::ActuationModelAbstract> mc_actuation, const int& frame_id,
                 const double& dt);
  ~ProblemMission();

  boost::shared_ptr<crocoddyl::ShootingProblem> createProblem();

 private:
  boost::shared_ptr<Mission> mission_;
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<pinocchio::Model> mc_model_;
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation_;

  int frame_id_;
  double dt_;
};
}  // namespace multicopter_mpc

#endif