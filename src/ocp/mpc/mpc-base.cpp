#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {
MpcAbstract::MpcAbstract(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                         const boost::shared_ptr<Mission>& mission)
    : OcpAbstract(model, mc_params) {
  assert(mission->getWaypoints().size() > 0);

  n_knots_ = 100;
  trajectory_generator_ = boost::make_shared<TrajectoryGenerator>(model_, mc_params_, mission);

  trajectory_generator_specs_.yaml_path = "";
  trajectory_generator_specs_.initial_guess = "";
  trajectory_generator_specs_.integrator = IntegratorTypes::NbIntegratorTypes;
  trajectory_generator_specs_.solver = SolverTypes::NbSolverTypes;
}

MpcAbstract::~MpcAbstract() {}

void MpcAbstract::loadParameters(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

  try {
    std::string solver = server.getParam<std::string>("trajectory_generator/solver");
    if (solver == "BoxFDDP") {
      trajectory_generator_specs_.solver = SolverTypes::BoxFDDP;
    } else if (solver == "BoxDDP") {
      trajectory_generator_specs_.solver = SolverTypes::BoxDDP;
    } else if (solver == "sbFDDP") {
      trajectory_generator_specs_.solver = SolverTypes::SquashBoxFDDP;
    } else {
      trajectory_generator_specs_.solver = SolverTypes::NbSolverTypes;
    }
  } catch (const std::exception& e) {
    MMPC_WARN << "TRAJECTORY GENERATOR PARAMS. Solver type not found.";
  }

  try {
    std::string integrator = server.getParam<std::string>("trajectory_generator/integrator");
    if (integrator == "Euler") {
      trajectory_generator_specs_.integrator = IntegratorTypes::Euler;
    } else if (integrator == "RK4") {
      trajectory_generator_specs_.integrator = IntegratorTypes::RK4;
    } else {
      trajectory_generator_specs_.integrator = IntegratorTypes::NbIntegratorTypes;
    }
  } catch (const std::exception& e) {
    MMPC_WARN << "TRAJECTORY GENERATOR PARAMS. Integrator type not found.";
  }

  try {
    std::string path = server.getParam<std::string>("trajectory_generator/yaml_path");
    trajectory_generator_specs_.yaml_path = path;
  } catch (const std::exception& e) {
    MMPC_WARN << "TRAJECTORY GENERATOR PARAMS. yaml_path not found.";
  }

  try {
    n_knots_ = server.getParam<int>("ocp/n_knots");
    MMPC_INFO << "MPC CONTROLLER PARAMS. n_knots set to: " << n_knots_;
  } catch (const std::exception& e) {
    MMPC_WARN << "MPC CONTROLLER PARAMS. n_knots not found, set to default: " << n_knots_;
  }
}

void MpcAbstract::initializeTrajectoryGenerator() {
  if (trajectory_generator_specs_.yaml_path != "") {
    trajectory_generator_->loadParameters(trajectory_generator_specs_.yaml_path);
  }

  SolverTypes::Type solver = (trajectory_generator_specs_.solver != SolverTypes::NbSolverTypes)
                                 ? trajectory_generator_specs_.solver
                                 : solver_type_;
  IntegratorTypes::Type integrator = (trajectory_generator_specs_.integrator != IntegratorTypes::NbIntegratorTypes)
                                         ? trajectory_generator_specs_.integrator
                                         : integrator_type_;
  trajectory_generator_->createProblem(solver, integrator, dt_);
  trajectory_generator_->setSolverIters(300);
  trajectory_generator_->setSolverCallbacks(true);
  std::vector<Eigen::VectorXd> state_trajectory =
      trajectory_generator_->getMission()->interpolateTrajectory(trajectory_generator_specs_.initial_guess);
  std::vector<Eigen::VectorXd> control_trajectory(trajectory_generator_->getKnots() - 1,
                                                  Eigen::VectorXd::Zero(actuation_->get_nu()));
  trajectory_generator_->solve(state_trajectory, control_trajectory);
  generateMission();
}

void MpcAbstract::generateMission() { mission_ = trajectory_generator_->getMission(); }

void MpcAbstract::setNumberKnots(const std::size_t& n_knots) {
  n_knots_ = n_knots;

  if (problem_ != nullptr) {
    diff_models_running_.clear();
    diff_model_terminal_ = nullptr;

    int_models_running_.clear();
    int_model_terminal_ = nullptr;

    problem_ = nullptr;
    solver_ = nullptr;
  }
}

const Eigen::VectorXd& MpcAbstract::getControls(const std::size_t& idx) const { return solver_->get_us()[idx]; }
const Eigen::VectorXd& MpcAbstract::getFeedForwardGains(const std::size_t& idx) const { return solver_->get_k()[idx]; }
const Eigen::MatrixXd& MpcAbstract::getFeedBackGains(const std::size_t& idx) const { return solver_->get_K()[idx]; };
const boost::shared_ptr<TrajectoryGenerator> MpcAbstract::getTrajectoryGenerator() const {
  return trajectory_generator_;
}
const boost::shared_ptr<Mission> MpcAbstract::getMission() const { return mission_; }

void MpcAbstract::printInfo() {}

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