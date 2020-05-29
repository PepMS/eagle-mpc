#ifndef MULTICOPTER_MPC_OCP_LOW_LEVEL_CONTROLLER_HPP_
#define MULTICOPTER_MPC_OCP_LOW_LEVEL_CONTROLLER_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "multicopter_mpc/ocp-base.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

class LowLevelController : public OcpAbstract {
 public:
  LowLevelController(const boost::shared_ptr<pinocchio::Model> model,
                      const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt);
  ~LowLevelController();

  virtual void createProblem(const SolverTypes::Type& solver_type);

 private:

};
}  // namespace multicopter_mpc

#endif
