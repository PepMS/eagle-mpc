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

FactoryMpc::FactoryMpc() {}
FactoryMpc::~FactoryMpc() {}

using createMethod = boost::shared_ptr<MpcAbstract> (*)(const boost::shared_ptr<pinocchio::Model>&,
                                                        const boost::shared_ptr<MultiCopterBaseParams>&, const double&,
                                                        const boost::shared_ptr<Mission>&, const std::size_t&);

FactoryMpc& FactoryMpc::get() {
  static FactoryMpc instance;
  return instance;
}

bool FactoryMpc::registerMpcController(const std::string& mpc_name, createMethod create_method) {
  auto it = s_methods_.find(mpc_name);
  if (it == s_methods_.end()) {
    s_methods_[mpc_name] = create_method;
    return true;
  }
  return false;
}

boost::shared_ptr<MpcAbstract> FactoryMpc::createMpcController(
    const std::string& mpc_name, const boost::shared_ptr<pinocchio::Model>& model,
    const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
    const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots) {
  auto it = s_methods_.find(mpc_name);
  if (it != s_methods_.end()) {
    return it->second(model, mc_params, dt, mission, n_knots);
  }
  return nullptr;
}

}  // namespace multicopter_mpc