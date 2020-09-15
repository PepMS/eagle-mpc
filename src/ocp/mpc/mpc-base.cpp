#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"

namespace multicopter_mpc {
MpcAbstract::MpcAbstract(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                         const boost::shared_ptr<Mission>& mission)
    : OcpAbstract(model, mc_params) {
  assert(mission->getWaypoints().size() > 0);

  n_knots_ = 100;
  trajectory_generator_ = boost::make_shared<TrajectoryGenerator>(model_, mc_params_, mission);

  trajectory_generator_specs_.yaml_path = "";
  trajectory_generator_specs_.initial_guess_type = "";
}

MpcAbstract::~MpcAbstract() {}

void MpcAbstract::initializeTrajectoryGenerator() {
  if (trajectory_generator_specs_.yaml_path != "") {
    trajectory_generator_->loadParameters(trajectory_generator_specs_.yaml_path);
  }
  trajectory_generator_->createProblem(solver_type_, integrator_type_, dt_);
  trajectory_generator_->setSolverIters(300);
  std::vector<Eigen::VectorXd> state_trajectory =
      trajectory_generator_->getMission()->interpolateTrajectory(trajectory_generator_specs_.initial_guess_type);
  std::vector<Eigen::VectorXd> control_trajectory(trajectory_generator_->getKnots() - 1,
                                                  Eigen::VectorXd::Zero(actuation_->get_nu()));
  trajectory_generator_->solve(state_trajectory, control_trajectory);
  generateMission();
}

void MpcAbstract::generateMission() {
  mission_ = trajectory_generator_->getMission();
}

const Eigen::VectorXd& MpcAbstract::getControls(const std::size_t& idx) const { return solver_->get_us()[idx]; }
const Eigen::VectorXd& MpcAbstract::getFeedForwardGains(const std::size_t& idx) const { return solver_->get_k()[idx]; }
const Eigen::MatrixXd& MpcAbstract::getFeedBackGains(const std::size_t& idx) const { return solver_->get_K()[idx]; };
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
    const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const boost::shared_ptr<Mission>& mission) {
  auto it = s_methods_.find(mpc_name);
  if (it != s_methods_.end()) {
    return it->second(model, mc_params, mission);
  }
  return nullptr;
}

}  // namespace multicopter_mpc