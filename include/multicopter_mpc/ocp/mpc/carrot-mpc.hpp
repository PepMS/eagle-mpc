#ifndef MULTICOPTER_MPC_OCP_CARROT_MPC_HPP_
#define MULTICOPTER_MPC_OCP_CARROT_MPC_HPP_

#include <algorithm>

#include <pinocchio/spatial/se3.hpp>

#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {

class CarrotMpc : public MpcAbstract {
 public:
  CarrotMpc(const boost::shared_ptr<pinocchio::Model>& model,
            const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
            const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);
  ~CarrotMpc();

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

  const bool existsTerminalWeight();

  const crocoddyl::FramePlacement& getPoseRef() const;
  const crocoddyl::FrameMotion& getVelocityRef() const;
  const TrajectoryGeneratorParams& getParams() const;
  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;

 protected:
  void initializeDefaultParameters() override;
  void initializeTrajectoryGenerator(const SolverTypes::Type& solver_type,
                                     const IntegratorTypes::Type& integrator_type) override;
  void initializeTerminalWeights();

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> createDifferentialModel(
      const std::size_t& idx_knot);
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> createCostControlRegularization();

  void setReference(const std::size_t& idx_trajectory);

  bool has_motion_ref_;
  Eigen::VectorXd state_ref_;
  Eigen::Quaterniond quat_ref_;
  crocoddyl::FramePlacement pose_ref_;
  crocoddyl::FrameMotion motion_ref_;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>>::iterator diff_model_iter_;

  std::vector<bool> terminal_weights_idx_;
  TrajectoryGeneratorParams params_;

 private:
  static bool registered_;
};
}  // namespace multicopter_mpc

#endif
