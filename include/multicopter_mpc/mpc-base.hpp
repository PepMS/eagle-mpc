#ifndef MULTICOPTER_MPC_MPC_BASE_HPP_
#define MULTICOPTER_MPC_MPC_BASE_HPP_

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/diff-action-base.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/factory/cost.hpp"
#include "multicopter_mpc/factory/int-action.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

enum class SolverTypes { SolverSbFDDP, SolverBoxFDDP, SolverBoxDDP };

static std::map<std::string, SolverTypes> SolverTypes_init_map() {
  std::map<std::string, SolverTypes> m;
  m.clear();
  m.insert({"SolverSbFDDP", SolverTypes::SolverSbFDDP});
  m.insert({"SolverBoxFDDP", SolverTypes::SolverBoxFDDP});
  m.insert({"SolverBoxDDP", SolverTypes::SolverBoxDDP});
  return m;
}
static const std::map<std::string, SolverTypes> SolverTypes_map = SolverTypes_init_map();

class MpcAbstract {
 public:
  MpcAbstract(const boost::shared_ptr<ParamsServer>& server);

 protected:
  virtual void createProblem() = 0;

  boost::shared_ptr<pinocchio::Model> robot_model_;
  boost::shared_ptr<MultiCopterBaseParams> platform_params_;
  std::string robot_model_path_;

  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squash_;
  boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation_squash_;

  boost::shared_ptr<crocoddyl::CostModelSum> costs_;

  boost::shared_ptr<CostModelFactory> cost_factory_;

  struct MpcParams {
    IntegratedActionModelTypes integrator_type;
    SolverTypes solver_type;
    std::size_t knots;
    std::size_t iters;
  } params_;

 private:
  void initializeRobotObjects(const boost::shared_ptr<ParamsServer>& server);
  void loadParams(const boost::shared_ptr<ParamsServer>& server);
};
}  // namespace multicopter_mpc

#endif