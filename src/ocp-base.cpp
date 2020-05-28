#include "ocp-base.hpp"

namespace multicopter_mpc {

OcpAbstract::OcpAbstract(const boost::shared_ptr<pinocchio::Model> model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const size_t& frame_base_link_id)
    : model_(model), mc_params_(mc_params), frame_base_link_id_(frame_base_link_id) {
  state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);

  state_weights_ = Eigen::VectorXd::Zero(state_->get_nx());

  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state_, mc_params_->n_rotors_, mc_params_->tau_f_);
}

OcpAbstract::~OcpAbstract() {}

boost::shared_ptr<crocoddyl::CostModelAbstract>& OcpAbstract::setCostStateRegularization() {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3).fill(1.);                        // Position 1.
  state_weights.segment(3, 3).fill(1.);                  // Orientation 1.
  state_weights.segment(model_->nv, 3).fill(1.);         // Linear velocity 1.
  state_weights.segment(model_->nv + 3, 3).fill(1000.);  // Angular velocity 1000.

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights_);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state =
      boost::make_shared<crocoddyl::CostModelState>(state_, activation_state, state_->zero(), actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract>& OcpAbstract::setCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

}  // namespace multicopter_mpc