#ifndef MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_CONTROLLER_HPP_
#define MULTICOPTER_MPC_OCP_TRAJECTORY_GENERATOR_CONTROLLER_HPP_

#include "multicopter_mpc/ocp-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {

class TrajectoryGeneratorController : public OcpAbstract {
 public:
  TrajectoryGeneratorController(const boost::shared_ptr<pinocchio::Model>& model,
                                const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);
  ~TrajectoryGeneratorController();

  void loadParameters(const yaml_parser::ParamsServer& server) override;
  void createProblem(const SolverTypes::Type& solver_type);
  void solve() override;

  // void updateReferences(const Eigen::Ref<Eigen::VectorXd>& state_new);
  void updateProblem(const std::size_t idx_trajectory);

  const crocoddyl::FramePlacement& getPoseRef() const;
  const crocoddyl::FrameMotion& getVelocityRef() const;
  const TrajectoryGeneratorParams& getParams() const;
  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const boost::shared_ptr<const Mission> getMission() const;

 protected:
  void initializeDefaultParameters() override;

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createRunningDifferentialModel();
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createTerminalDifferentialModel();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void updateWeights(const std::size_t& idx_trajectory);
  void updateTerminalCost(const std::size_t idx_trajectory);

  void setPoseRef(const std::size_t& idx_trajectory);
  void setMotionRef(const std::size_t& idx_trajectory);

  boost::shared_ptr<Mission> mission_;

  bool has_motion_ref_;
  crocoddyl::FramePlacement pose_ref_;
  crocoddyl::FrameMotion motion_ref_;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>::iterator diff_model_iter_;

  std::vector<std::size_t> terminal_weights_idx_;
  TrajectoryGeneratorParams params_;
};
}  // namespace multicopter_mpc

#endif