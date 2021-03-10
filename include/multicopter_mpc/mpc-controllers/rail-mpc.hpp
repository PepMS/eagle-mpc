#ifndef MULTICOPTER_MPC_MPC_CONTROLLERS_RAIL_MPC_HPP_
#define MULTICOPTER_MPC_MPC_CONTROLLERS_RAIL_MPC_HPP_

// #include "pinocchio/algorithm/joint-configuration.hpp"

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/core/costs/control.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/mpc-base.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

class RailMpc : public MpcAbstract {
 public:
  RailMpc(const std::vector<Eigen::VectorXd>& state_ref, const std::size_t dt_ref, const std::string& yaml_path);

  virtual ~RailMpc();

  virtual void createProblem() override;
  virtual void updateProblem(const std::size_t& current_time) override;

  const std::vector<Eigen::VectorXd>& get_state_ref() const;
  const std::vector<std::size_t>& get_t_ref() const;

 private:
  boost::shared_ptr<crocoddyl::CostModelSum> createCosts() const;
  void updateContactCosts(const std::size_t& idx);
  void updateFreeCosts(const std::size_t& idx);
  void computeStateReference(const std::size_t& time);

  std::vector<Eigen::VectorXd> state_ref_;
  std::vector<std::size_t> t_ref_;

  Eigen::VectorXd state_activation_weights_;
  double state_weight_;
  double control_weight_;

  struct UpdateVars {
    std::size_t idx_stage;
    std::size_t idx_last_stage;
    std::size_t node_time;
    std::string name_stage;
    boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> dif_contact;
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> dif_free;
    Eigen::VectorXd state_ref;
    std::size_t idx_state;
    double alpha;
    Eigen::Quaterniond quat_hover;
  } update_vars_;
};
}  // namespace multicopter_mpc

#endif
