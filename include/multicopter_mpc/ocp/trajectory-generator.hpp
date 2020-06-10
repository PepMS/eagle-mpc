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

  void createProblem(const SolverTypes::Type& solver_type) override;

  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void solve() override;

  const boost::shared_ptr<const Mission> getMission() const;
  std::vector<Eigen::VectorXd> getTrajectoryPortion(const std::size_t& idx_init, const std::size_t& idx_end) const;
  const Eigen::VectorXd& getTrajectoryState(const std::size_t& cursor) const;

 private:
  boost::shared_ptr<Mission> mission_;
  std::vector<Eigen::VectorXd> state_trajectory_;
  Eigen::VectorXd state_hover_;
};
}  // namespace multicopter_mpc

#endif
