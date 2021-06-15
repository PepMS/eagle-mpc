///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "eagle_mpc/mpc-controllers/rail-mpc.hpp"
#include "eagle_mpc/utils/log.hpp"

namespace eagle_mpc {

RailMpc::RailMpc(const std::vector<Eigen::VectorXd>& state_ref, const std::size_t dt_ref, const std::string& yaml_path)
    : MpcAbstract(yaml_path) {
  state_ref_ = std::vector<Eigen::VectorXd>(state_ref.size(), robot_state_->zero());
  std::copy(state_ref.begin(), state_ref.end(), state_ref_.begin());
  for (std::size_t i = 0; i < state_ref_.size(); ++i) {
    t_ref_.push_back(dt_ref * i);
  }

  try {
    state_weight_ = params_server_->getParam<double>("mpc_controller/rail_weight");
  } catch (const std::exception& e) {
    MMPC_WARN << "The following key: 'mpc_controller/rail_weight' has not been found in the parameters server. Set "
                 "to 10.0";
    state_weight_ = 10;
  }

  try {
    state_activation_weights_ = converter<Eigen::VectorXd>::convert(
        params_server_->getParam<std::string>("mpc_controller/rail_activation_weights"));
  } catch (const std::exception& e) {
    MMPC_WARN << "The following key: 'mpc_controller/rail_activation_weights' has not been found in the parameters "
                 "server. Set "
                 "to unitary vector";
    state_activation_weights_ = Eigen::VectorXd::Ones(robot_state_->get_ndx());
  }
  if (state_activation_weights_.size() != robot_state_->get_ndx()) {
    std::runtime_error("RailMPC: the dimension for the state activation weights vector is " +
                       std::to_string(state_activation_weights_.size()) + ", should be " +
                       std::to_string(robot_state_->get_ndx()));
  }

  try {
    control_weight_ = params_server_->getParam<double>("mpc_controller/rail_control_weight");
  } catch (const std::exception& e) {
    MMPC_WARN
        << "The following key: 'mpc_controller/rail_control_weight' has not been found in the parameters server. Set "
           "to 1e-1";
    control_weight_ = 1e-1;
  }
  createProblem();

  update_vars_.state_ref = robot_state_->zero();
}

RailMpc::~RailMpc() {}

void RailMpc::createProblem() {
  DifferentialActionModelTypes dif_type;
  dif_type = DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics;

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
      solver_ = boost::make_shared<eagle_mpc::SolverSbFDDP>(problem_, squash_);
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

boost::shared_ptr<crocoddyl::CostModelSum> RailMpc::createCosts() const {
  boost::shared_ptr<crocoddyl::CostModelSum> costs =
      boost::make_shared<crocoddyl::CostModelSum>(robot_state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_activation_weights_);
  boost::shared_ptr<crocoddyl::CostModelState> rail_cost = boost::make_shared<crocoddyl::CostModelState>(
      robot_state_, activation, robot_state_->zero(), actuation_->get_nu());
  costs->addCost("rail_state", rail_cost, state_weight_, true);

  boost::shared_ptr<crocoddyl::CostModelControl> control_cost =
      boost::make_shared<crocoddyl::CostModelControl>(robot_state_, actuation_->get_nu());
  costs->addCost("control", control_cost, control_weight_, true);

  return costs;
}

void RailMpc::updateProblem(const std::size_t& current_time) {
  for (std::size_t i = 0; i < dif_models_.size(); ++i) {
    update_vars_.node_time = current_time + i * params_.dt;
    // if (trajectory_->get_has_contact()) {
    //   updateContactCosts(i);
    // } else {
    updateFreeCosts(i);
    // }
  }
}

void RailMpc::updateContactCosts(const std::size_t& idx) {}

void RailMpc::updateFreeCosts(const std::size_t& idx) {
  update_vars_.dif_free =
      boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(dif_models_[idx]);

  computeStateReference(update_vars_.node_time);
  update_vars_.dif_free->get_costs()->get_costs().at("rail_state")->cost->set_reference(update_vars_.state_ref);
}

void RailMpc::computeStateReference(const std::size_t& time) {
  update_vars_.idx_state = std::size_t(std::upper_bound(t_ref_.begin(), t_ref_.end(), time) - t_ref_.begin());
  if (update_vars_.idx_state >= state_ref_.size()) {
    update_vars_.state_ref = robot_state_->zero();
    update_vars_.state_ref.head(robot_model_->nq) = state_ref_.back().head(robot_model_->nq);
    update_vars_.quat_hover = Eigen::Quaterniond(state_ref_.back()(6), 0.0, 0.0, state_ref_.back()(5));
    update_vars_.quat_hover.normalize();
    update_vars_.state_ref(5) = update_vars_.quat_hover.z();
    update_vars_.state_ref(6) = update_vars_.quat_hover.w();
  } else {
    update_vars_.alpha = (time - t_ref_[update_vars_.idx_state - 1]) /
                         (t_ref_[update_vars_.idx_state] - t_ref_[update_vars_.idx_state - 1]);
    update_vars_.state_ref.head(robot_model_->nq) =
        pinocchio::interpolate(*robot_model_, state_ref_[update_vars_.idx_state - 1].head(robot_model_->nq),
                               state_ref_[update_vars_.idx_state].head(robot_model_->nq), update_vars_.alpha);
    update_vars_.state_ref.tail(robot_model_->nv) =
        state_ref_[update_vars_.idx_state - 1].tail(robot_model_->nv) +
        update_vars_.alpha * (state_ref_[update_vars_.idx_state].tail(robot_model_->nv) -
                              state_ref_[update_vars_.idx_state - 1].tail(robot_model_->nv));
  }
}

const std::vector<Eigen::VectorXd>& RailMpc::get_state_ref() const { return state_ref_; }
const std::vector<std::size_t>& RailMpc::get_t_ref() const { return t_ref_; }

}  // namespace eagle_mpc
