#ifndef MULTICOPTER_MPC_OCP_RAIL_MPC_HPP_
#define MULTICOPTER_MPC_OCP_RAIL_MPC_HPP_

#include <cassert>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "yaml_parser/params_server.hpp"
#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"

namespace multicopter_mpc {

struct RailMpcParams {
  double w_state;                        // General penalization for the state error
  Eigen::Vector3d w_state_position;      // Importance of the position error in the cost function
  Eigen::Vector3d w_state_orientation;   // Importance of the orientation error in the cost function
  Eigen::Vector3d w_state_velocity_lin;  // Importance of the linear velocity error in the cost function
  Eigen::Vector3d w_state_velocity_ang;  // Importance of the angular velocity in the cost function

  double w_control;  // General penalization for the control
};

class RailMpc : public MpcAbstract {
 public:
  RailMpc(const boost::shared_ptr<pinocchio::Model>& model, const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
          const double& dt, const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);
  ~RailMpc();

  static std::string getFactoryName();
  static boost::shared_ptr<MpcAbstract> createMpcController(const boost::shared_ptr<pinocchio::Model>& model,
                                                            const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                                            const double& dt,
                                                            const boost::shared_ptr<Mission>& mission,
                                                            const std::size_t& n_knots);

  void loadParameters(const std::string& yaml_path) override;
  void createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type) override;
  void setTimeStep(const double& dt) override;

  void solve() override;

  void updateProblem(const std::size_t idx_trajectory);

  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const std::vector<Eigen::VectorXd>& getStateRef() const;
  const RailMpcParams& getParams() const;
  void setReferences(const std::vector<Eigen::VectorXd>& state_trajectory,
                     const std::vector<Eigen::VectorXd>& control_trajectory);

  void printCosts();

 protected:
  void initializeTrajectoryGenerator(const SolverTypes::Type& solver_type,
                                     const IntegratorTypes::Type& integrator_type) override;
  void initializeDefaultParameters() override;

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createDifferentialModel(
      const unsigned int& trajectory_idx);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostState(const unsigned int& trajectory_idx);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControl(const unsigned int& trajectory_idx);

  std::vector<Eigen::VectorXd> state_ref_;    // Vector containing the state reference for each node
  std::vector<Eigen::VectorXd> control_ref_;  // Vector containing the control reference for each node

  RailMpcParams params_;

 private:
  static bool registered_;
};
}  // namespace multicopter_mpc

#endif
