#ifndef MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_CONTROLLER_HPP_
#define MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_CONTROLLER_HPP_

#include "multicopter_mpc/ocp-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

class TrajectoryGeneratorController : public OcpAbstract {
 public:
  TrajectoryGeneratorController(const boost::shared_ptr<pinocchio::Model>& model,
                                const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                const std::size_t& n_knots);
  ~TrajectoryGeneratorController();

  void loadParameters(const yaml_parser::ParamsServer& server) override;
  void createProblem(const SolverTypes::Type& solver_type);
  void solve() override;

  void updateReferences(const Eigen::Ref<Eigen::VectorXd>& state_new);

  const crocoddyl::FramePlacement& getPoseRef() const;
  const crocoddyl::FrameMotion& getVelocityRef() const;
  const TrajectoryGeneratorParams& getParams() const;
  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;

 private:
  void initializeDefaultParameters() override;

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createDifferentialRunningModel();
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createDifferentialTerminalModel();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void setPoseRef();
  void setMotionRef();

  Eigen::Vector3d pos_ref_;
  Eigen::Quaterniond quat_ref_;
  Eigen::Vector3d vel_lin_ref_;
  Eigen::Vector3d vel_ang_ref_;

  crocoddyl::FramePlacement pose_ref_;
  crocoddyl::FrameMotion motion_ref_;

  TrajectoryGeneratorParams params_;
};
}  // namespace multicopter_mpc

#endif