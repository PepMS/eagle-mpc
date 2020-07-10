#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"

namespace multicopter_mpc {

MpcAbstract::MpcAbstract(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                         const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots)
    : OcpAbstract(model, mc_params, dt) {
  assert(mission->getWaypoints().size() > 0);
  n_knots_ = n_knots;

  trajectory_generator_ = boost::make_shared<TrajectoryGenerator>(model_, mc_params_, dt_, mission);
}

MpcAbstract::~MpcAbstract() {}

const Eigen::VectorXd& MpcAbstract::getControls(const std::size_t& idx) const { return solver_->get_us()[idx]; }
const boost::shared_ptr<TrajectoryGenerator> MpcAbstract::getTrajectoryGenerator() const {
  return trajectory_generator_;
}
const boost::shared_ptr<Mission> MpcAbstract::getMission() const { return mission_; }

}  // namespace multicopter_mpc