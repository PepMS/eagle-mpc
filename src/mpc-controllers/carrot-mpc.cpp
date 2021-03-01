#include "multicopter_mpc/mpc-controllers/carrot-mpc.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

CarrotMpc::CarrotMpc(const boost::shared_ptr<Trajectory>& trajectory, const std::vector<Eigen::VectorXd>& state_ref,
                     const std::size_t dt_ref, const std::string& yaml_path)
    : MpcAbstract(yaml_path), trajectory_(trajectory) {
  createProblem();

  state_ref_ = std::vector<Eigen::VectorXd>(state_ref.size(), robot_state_->zero());
  std::copy(state_ref.begin(), state_ref.end(), state_ref_.begin());
  for (std::size_t i = 0; i < state_ref_.size(); ++i) {
    t_ref_.push_back(dt_ref*i);
  }
}

CarrotMpc::~CarrotMpc() {}

void CarrotMpc::createProblem() {
  DifferentialActionModelTypes dif_type;
  if (trajectory_->get_has_contact()) {
    dif_type = DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics;
  } else {
    dif_type = DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics;
  }

  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  if (params_.solver_type == SolverTypes::SolverSbFDDP) {
    actuation = actuation_squash_;
  } else {
    actuation = actuation_;
  }

  for (std::size_t i = 0; i < params_.knots; ++i) {
    boost::shared_ptr<crocoddyl::CostModelSum> costs = createCosts();

    boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam;
    switch (dif_type) {
      case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics:
        dam = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(robot_state_, actuation, costs);
        break;
      case DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics:
        MMPC_ERROR << "Carrot with contact has not been implemented";
        break;
    }

    boost::shared_ptr<crocoddyl::ActionModelAbstract> iam;
    double dt_s = double(10) / 1000.;
    switch (params_.integrator_type) {
      case IntegratedActionModelTypes::IntegratedActionModelEuler:
        iam = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(dam, dt_s);
        break;

      case IntegratedActionModelTypes::IntegratedActionModelRK4:
        iam = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(dam, dt_s);
        break;
    }

    dif_models_.push_back(dam);
    int_models_.push_back(iam);
  }

  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(
      robot_state_->zero(),
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(int_models_.begin(), int_models_.end() - 1),
      int_models_.back());
}

boost::shared_ptr<crocoddyl::CostModelSum> CarrotMpc::createCosts() const {
  boost::shared_ptr<crocoddyl::CostModelSum> costs =
      boost::make_shared<crocoddyl::CostModelSum>(robot_state_, actuation_->get_nu());

  for (auto stage = trajectory_->get_stages().begin(); stage != trajectory_->get_stages().end(); ++stage) {
    std::string path_to_stage = "stages/" + (*stage)->get_name();
    for (auto ctype = (*stage)->get_cost_types().begin(); ctype != (*stage)->get_cost_types().end(); ++ctype) {
      std::string cost_name = ctype->first;
      CostModelTypes cost_type = ctype->second;
      boost::shared_ptr<crocoddyl::CostModelAbstract> cost =
          cost_factory_->create(path_to_stage + "/costs/" + cost_name + "/", trajectory_->get_params_server(),
                                robot_state_, actuation_->get_nu(), cost_type);
      costs->addCost((*stage)->get_name() + "/" + cost_name, cost,
                     (*stage)->get_costs()->get_costs().at(cost_name)->weight, false);
    }
  }

  boost::shared_ptr<crocoddyl::CostModelState> carrot_cost =
      boost::make_shared<crocoddyl::CostModelState>(robot_state_, robot_state_->zero(), actuation_->get_nu());
  costs->addCost("state", carrot_cost, 10, false);

  return costs;
}

const boost::shared_ptr<Trajectory>& CarrotMpc::get_trajectory() const { return trajectory_; }
const std::vector<Eigen::VectorXd>& CarrotMpc::get_state_ref() const { return state_ref_; }
const std::vector<std::size_t>& CarrotMpc::get_t_ref() const { return t_ref_; }

}  // namespace multicopter_mpc
