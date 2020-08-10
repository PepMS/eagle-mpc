#ifndef MULTICOPTER_MPC_OCP_PICEWISE_MPC_HPP_
#define MULTICOPTER_MPC_OCP_PICEWISE_MPC_HPP_

#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {

class PiceWiseMpc : public MpcAbstract {
 public:
  PiceWiseMpc(const boost::shared_ptr<pinocchio::Model>& model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
              const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);
  ~PiceWiseMpc();

  static std::string getFactoryName();
  static boost::shared_ptr<MpcAbstract> createMpcController(const boost::shared_ptr<pinocchio::Model>& model,
                                                            const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                                            const double& dt,
                                                            const boost::shared_ptr<Mission>& mission,
                                                            const std::size_t& n_knots);

  void loadParameters(const std::string& yaml_path) override;
  void createProblem(const SolverTypes::Type& solver_type) override;
  void setTimeStep(const double& dt) override;

  void solve() override;

  void updateProblem(const std::size_t idx_trajectory);

  const crocoddyl::FramePlacement& getPoseRef() const;
  const crocoddyl::FrameMotion& getVelocityRef() const;
  const TrajectoryGeneratorParams& getParams() const;
  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;

 protected:
  void initializeDefaultParameters() override;
  void initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) override;

  std::size_t splitWaypoint(const std::size_t& wp_original_knots);
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createRunningDifferentialModel(
      const std::size_t& idx_knot);
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createTerminalDifferentialModel(
      const std::size_t& idx_knot);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void updateWeights(const std::size_t& idx_trajectory);
  void updateTerminalCost(const std::size_t idx_trajectory);

  void setPoseRef(const std::size_t& idx_trajectory);
  void setMotionRef(const std::size_t& idx_trajectory);

  bool has_motion_ref_;
  crocoddyl::FramePlacement pose_ref_;
  crocoddyl::FrameMotion motion_ref_;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>::iterator diff_model_iter_;

  std::vector<std::size_t> terminal_weights_idx_;
  TrajectoryGeneratorParams params_;

 private:
  static bool registered_;
};
}  // namespace multicopter_mpc

#endif