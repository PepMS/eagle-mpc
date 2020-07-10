#ifndef MULTICOPTER_MPC_MPC_BASE_HPP_
#define MULTICOPTER_MPC_MPC_BASE_HPP_

#include <boost/shared_ptr.hpp>

#include "multicopter_mpc/ocp/ocp-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/mission.hpp"

namespace multicopter_mpc {

struct MpcTypes {
  enum Type { RailMpc, TgMpc };
};

class MpcAbstract : public OcpAbstract {
 public:
  MpcAbstract(const boost::shared_ptr<pinocchio::Model>& model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
              const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);

  ~MpcAbstract();

  virtual void updateProblem(const std::size_t idx_trajectory) = 0;

  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const boost::shared_ptr<TrajectoryGenerator> getTrajectoryGenerator() const;
  const boost::shared_ptr<Mission> getMission() const;

 protected:
  virtual void initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) = 0;

  boost::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  boost::shared_ptr<Mission> mission_;

};
}  // namespace multicopter_mpc

#endif