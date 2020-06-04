#ifndef MULTICOPTER_MPC_OCP_LOW_LEVEL_CONTROLLER_HPP_
#define MULTICOPTER_MPC_OCP_LOW_LEVEL_CONTROLLER_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "yaml_parser/params_server.hpp"
#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/ocp-base.hpp"

namespace multicopter_mpc {

struct LowLevelControllerParams {
  double w_state;                        // General penalization for the state error
  Eigen::Vector3d w_state_position;      // Importance of the position error in the cost function
  Eigen::Vector3d w_state_orientation;   // Importance of the orientation error in the cost function
  Eigen::Vector3d w_state_velocity_lin;  // Importance of the linear velocity error in the cost function
  Eigen::Vector3d w_state_velocity_ang;  // Importance of the angular velocity in the cost function

  double w_control;  // General penalization for the control
};

class LowLevelController : public OcpAbstract {
 public:
  LowLevelController(const boost::shared_ptr<pinocchio::Model>& model,
                     const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                     std::size_t& n_knots);
  ~LowLevelController();

  void loadParameters(const yaml_parser::ParamsServer& server) override;

  void createProblem(const SolverTypes::Type& solver_type);
  void solve() override;

  void updateReferenceStateTrajectory(const Eigen::Ref<Eigen::VectorXd>& state_new);

  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const std::vector<Eigen::VectorXd>& getStateRef() const;
  const LowLevelControllerParams& getParams() const;
  void setReferenceStateTrajectory(const std::vector<Eigen::VectorXd>& state_trajectory);

 private:
  void initializeDefaultParameters() override;

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createDifferentialModel(
      const unsigned int& trajectory_idx);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostState(const unsigned int& trajectory_idx);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  std::vector<Eigen::VectorXd> state_ref_;  // Vector containing the state reference for each node
  LowLevelControllerParams params_;
};
}  // namespace multicopter_mpc

#endif
