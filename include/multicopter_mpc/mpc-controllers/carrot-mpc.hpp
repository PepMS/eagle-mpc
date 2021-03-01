#ifndef MULTICOPTER_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_
#define MULTICOPTER_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/mpc-base.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

class CarrotMpc : public MpcAbstract {
 public:
  CarrotMpc(const boost::shared_ptr<Trajectory>& trajectory, const std::vector<Eigen::VectorXd>& state_ref,
            const std::size_t dt_ref, const std::string& yaml_path);

  virtual ~CarrotMpc();

  void createProblem() override;

  const boost::shared_ptr<Trajectory>& get_trajectory() const;
  const std::vector<Eigen::VectorXd>& get_state_ref() const;

 private:
  boost::shared_ptr<crocoddyl::CostModelSum> createCosts() const;

  boost::shared_ptr<Trajectory> trajectory_;
  std::vector<Eigen::VectorXd> state_ref_;
  std::size_t dt_ref_;
};
}  // namespace multicopter_mpc

#endif
