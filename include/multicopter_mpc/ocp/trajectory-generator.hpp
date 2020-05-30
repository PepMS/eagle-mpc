#ifndef MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_
#define MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "multicopter_mpc/ocp-base.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

class TrajectoryGenerator : public OcpAbstract {
 public:
  TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                      const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                      const boost::shared_ptr<Mission>& mission);
  ~TrajectoryGenerator();

  virtual void createProblem(const SolverTypes::Type& solver_type);

  virtual boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  virtual boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();
  
 private:
  boost::shared_ptr<Mission> mission_;
};
}  // namespace multicopter_mpc

#endif
