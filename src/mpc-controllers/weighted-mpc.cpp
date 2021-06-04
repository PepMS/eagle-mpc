#include <algorithm>
#include <cmath>

#include "multicopter_mpc/mpc-controllers/weighted-mpc.hpp"
#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

WeightedMpc::WeightedMpc(const boost::shared_ptr<Trajectory>& trajectory, const std::size_t dt_ref,
                         const std::string& yaml_path)
    : MpcAbstract(yaml_path), trajectory_(trajectory) {
  try {
    alpha_ = params_server_->getParam<double>("mpc_controller/weighted_alpha");
  } catch (const std::exception& e) {
    MMPC_WARN << "The following key: 'mpc_controller/weighted_alpha' has not been found in the parameters server. Set "
                 "to 20.0";
    alpha_ = 20.0;
  }

  try {
    beta_ = params_server_->getParam<double>("mpc_controller/weighted_beta");
  } catch (const std::exception& e) {
    MMPC_WARN << "The following key: 'mpc_controller/weighted_beta' has not been found in the parameters server. Set "
                 "to 1.0";
    beta_ = 1.0;
  }

  try {
    state_reg_ = params_server_->getParam<double>("mpc_controller/weighted_state_reg");
  } catch (const std::exception& e) {
    MMPC_WARN
        << "The following key: 'mpc_controller/weighted_state_reg' has not been found in the parameters server. Set "
           "to 1e-1";
    state_reg_ = 1e-1;
  }

  try {
    control_reg_ = params_server_->getParam<double>("mpc_controller/weighted_control_reg");
  } catch (const std::exception& e) {
    MMPC_WARN
        << "The following key: 'mpc_controller/weighted_control_reg' has not been found in the parameters server. Set "
           "to 1e-1";
    control_reg_ = 1e-1;
  }

  for (std::size_t i = 0; i < trajectory_->get_stages().size();) {
    if (trajectory_->get_stages()[i]->get_is_transition()) {
      trajectory_->get_stages()[i + 1]->set_duration(trajectory_->get_stages()[i]->get_duration() +
                                                     trajectory_->get_stages()[i + 1]->get_duration());
      trajectory_->get_stages()[i + 1]->set_t_ini(trajectory_->get_stages()[i]->get_t_ini());
      trajectory_->removeStage(i);
      t_stages_.push_back(trajectory_->get_stages()[i]->get_t_ini());
      ++i;
    } else {
      t_stages_.push_back(trajectory_->get_stages()[i]->get_t_ini());
      ++i;
    }
  }

  createProblem();
}

WeightedMpc::~WeightedMpc() {}

void WeightedMpc::createProblem() {
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
        MMPC_ERROR << "Weighted with contact has not been implemented";
        break;
    }

    boost::shared_ptr<crocoddyl::ActionModelAbstract> iam;
    double dt_s = double(params_.dt) / 1000.;
    switch (params_.integrator_type) {
      case IntegratedActionModelTypes::IntegratedActionModelEuler:
        iam = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(dam, dt_s);
        break;

      case IntegratedActionModelTypes::IntegratedActionModelRK4:
        iam = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(dam, dt_s);
        break;
    }
    iam->set_u_lb(platform_params_->u_lb);
    iam->set_u_ub(platform_params_->u_ub);

    dif_models_.push_back(dam);
    int_models_.push_back(iam);
  }

  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(
      robot_state_->zero(),
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(int_models_.begin(), int_models_.end() - 1),
      int_models_.back());

  switch (params_.solver_type) {
    case SolverTypes::SolverSbFDDP:
      solver_ = boost::make_shared<multicopter_mpc::SolverSbFDDP>(problem_, squash_);
      break;
    case SolverTypes::SolverBoxFDDP:
      solver_ = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem_);
      break;
    case SolverTypes::SolverBoxDDP:
      solver_ = boost::make_shared<crocoddyl::SolverBoxDDP>(problem_);
      break;
  }
  solver_callbacks_.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  solver_->setCallbacks(solver_callbacks_);
}

boost::shared_ptr<crocoddyl::CostModelSum> WeightedMpc::createCosts() const {
  boost::shared_ptr<crocoddyl::CostModelSum> costs =
      boost::make_shared<crocoddyl::CostModelSum>(robot_state_, actuation_->get_nu());

  for (auto stage = trajectory_->get_stages().begin(); stage != trajectory_->get_stages().end(); ++stage) {
    if ((*stage)->get_is_transition()) {
      continue;
    }
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

  return costs;
}

void WeightedMpc::updateProblem(const std::size_t& current_time) {
  computeActiveStage(current_time);
  update_vars_.idx_last_stage = update_vars_.idx_stage;
  for (std::size_t i = 0; i < dif_models_.size(); ++i) {
    update_vars_.node_time = current_time + i * params_.dt;
    computeActiveStage(update_vars_.node_time, update_vars_.idx_last_stage);
    update_vars_.name_stage = trajectory_->get_stages()[update_vars_.idx_stage]->get_name();
    if (trajectory_->get_has_contact()) {
      updateContactCosts(i);
    } else {
      updateFreeCosts(i);
    }
    update_vars_.idx_last_stage = update_vars_.idx_stage;
  }
}

void WeightedMpc::computeActiveStage(const std::size_t& current_time) {
  update_vars_.idx_stage =
      std::size_t(std::upper_bound(t_stages_.begin(), t_stages_.end(), current_time) - t_stages_.begin()) - 1;
}

void WeightedMpc::computeActiveStage(const std::size_t& current_time, const std::size_t& last_stage) {
  computeActiveStage(current_time);
  if (update_vars_.idx_stage == last_stage + 2) {
    update_vars_.idx_stage -= 1;
  }
}

void WeightedMpc::updateContactCosts(const std::size_t& idx) {}

void WeightedMpc::updateFreeCosts(const std::size_t& idx) {
  update_vars_.dif_free =
      boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(dif_models_[idx]);
  for (auto cost = update_vars_.dif_free->get_costs()->get_costs().begin();
       cost != update_vars_.dif_free->get_costs()->get_costs().end(); cost++) {
    if (cost->first.compare(0, update_vars_.name_stage.size(), update_vars_.name_stage) == 0) {
      cost->second->active = true;
      if (cost->first.compare(update_vars_.name_stage.size(), 4, "/reg") != 0 &&
          cost->first.compare(update_vars_.name_stage.size(), 7, "/limits") != 0) {
        computeWeight(update_vars_.node_time);
        // cost->second->weight = update_vars_.weight;
        cost->second->weight = trajectory_->get_stages()[update_vars_.idx_stage]
                                   ->get_costs()
                                   ->get_costs()
                                   .at(cost->first.substr(update_vars_.name_stage.size() + 1))
                                   ->weight *
                               update_vars_.weight * beta_;
      }
    } else {
      if (cost->first.compare(0, cost->first.size(), "barrier") != 0) {
        cost->second->active = false;
      }
    }
  }
}

void WeightedMpc::computeWeight(const std::size_t& time) {
  // Saturate the weight once the nodes are beyond the end of the trajectory
  if (time > trajectory_->get_duration()) {
    update_vars_.weight_time = 0.0;
  } else {
    update_vars_.weight_time = update_vars_.weight_time =
        ((int)time - ((int)trajectory_->get_stages()[update_vars_.idx_stage]->get_t_ini() +
                      (int)trajectory_->get_stages()[update_vars_.idx_stage]->get_duration())) /
        1000.0;
  }
  update_vars_.weight = exp(alpha_ * update_vars_.weight_time);
}

const boost::shared_ptr<Trajectory>& WeightedMpc::get_trajectory() const { return trajectory_; }
const std::vector<std::size_t>& WeightedMpc::get_t_stages() const { return t_stages_; }

}  // namespace multicopter_mpc
