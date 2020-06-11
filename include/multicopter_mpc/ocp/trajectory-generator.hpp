#ifndef MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_
#define MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "yaml_parser/params_server.hpp"
#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/ocp-base.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

struct TrajectoryGeneratorParams {
  Eigen::Vector3d w_state_position;      // Importance of the position in the state regularization
  Eigen::Vector3d w_state_orientation;   // Importance of the orientationin the state regularization
  Eigen::Vector3d w_state_velocity_lin;  // Importance of the linear velocity in the state regularization
  Eigen::Vector3d w_state_velocity_ang;  // Importance of the angular velocity in the state regularization

  double w_state_running; // General penalization for the state in running models
  double w_control_running; // General penalization for the control in running models
  double w_pos_running; // General penalization for the pose error in running models
  double w_vel_running; // General penalization for the pose error in running models
  
  double w_pos_terminal; // General penalization for the pose error in terminal model
  double w_vel_terminal; // General penalization for the pose error in terminal model

};

class TrajectoryGenerator : public OcpAbstract {
 public:
  TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                      const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                      const boost::shared_ptr<Mission>& mission);
  ~TrajectoryGenerator();

  void loadParameters(const yaml_parser::ParamsServer& server) override;

  void createProblem(const SolverTypes::Type& solver_type);

  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void solve() override;

  const boost::shared_ptr<const Mission> getMission() const;
  std::vector<Eigen::VectorXd> getTrajectoryPortion(const std::size_t& idx_init, const std::size_t& idx_end) const;
  const Eigen::VectorXd& getTrajectoryState(const std::size_t& cursor) const;

 private:
  void initializeDefaultParameters() override;

  boost::shared_ptr<Mission> mission_;
  std::vector<Eigen::VectorXd> state_trajectory_;
  Eigen::VectorXd state_hover_;
  TrajectoryGeneratorParams params_;
};
}  // namespace multicopter_mpc

#endif
