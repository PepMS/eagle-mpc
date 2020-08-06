#ifndef MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_
#define MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "yaml_parser/params_server.hpp"
#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/ocp/ocp-base.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

struct TrajectoryGeneratorParams {
  Eigen::Vector3d w_state_position;      // Importance of the position in the state regularization
  Eigen::Vector3d w_state_orientation;   // Importance of the orientationin the state regularization
  Eigen::Vector3d w_state_velocity_lin;  // Importance of the linear velocity in the state regularization
  Eigen::Vector3d w_state_velocity_ang;  // Importance of the angular velocity in the state regularization

  double w_state_running;    // General penalization for the state in running models
  double w_control_running;  // General penalization for the control in running models
  double w_pos_running;      // General penalization for the pose error in running models
  double w_vel_running;      // General penalization for the pose error in running models

  double w_pos_terminal;  // General penalization for the pose error in terminal model
  double w_vel_terminal;  // General penalization for the pose error in terminal model
};

class TrajectoryGenerator : public OcpAbstract {
 public:
  TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                      const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                      const boost::shared_ptr<Mission>& mission);
  ~TrajectoryGenerator();

  void loadParameters(const std::string& yaml_path) override;

  void createProblem(const SolverTypes::Type& solver_type) override;

  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void solve() override;
  void solve(const std::vector<Eigen::VectorXd>& state_trajectory,
             const std::vector<Eigen::VectorXd>& control_trajectory) override;

  const boost::shared_ptr<Mission> getMission() const;
  std::vector<Eigen::VectorXd> getStateTrajectory(const std::size_t& idx_init, const std::size_t& idx_end) const;
  std::vector<Eigen::VectorXd> getControlTrajectory(const std::size_t& idx_init, const std::size_t& idx_end) const;
  const Eigen::VectorXd& getState(const std::size_t& cursor) const;
  const Eigen::VectorXd& getControl(const std::size_t& cursor) const;
  const boost::shared_ptr<const crocoddyl::SolverAbstract> getSolver() const;
  const TrajectoryGeneratorParams& getParams() const;

 private:
  void initializeDefaultParameters() override;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createRunningDifferentialModel(
      const WayPoint& waypoint);
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createTerminalDifferentialModel(
      const WayPoint& waypoint, const bool& is_last_wp);

  boost::shared_ptr<Mission> mission_;
  // std::vector<Eigen::VectorXd> state_trajectory_;
  // std::vector<Eigen::VectorXd> control_trajectory_;
  Eigen::VectorXd state_hover_;
  Eigen::VectorXd control_hover_;
  TrajectoryGeneratorParams params_;
};
}  // namespace multicopter_mpc

#endif
