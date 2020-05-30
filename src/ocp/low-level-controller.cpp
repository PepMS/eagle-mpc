#include "multicopter_mpc/ocp/low-level-controller.hpp"

namespace multicopter_mpc {

LowLevelController::LowLevelController(const boost::shared_ptr<pinocchio::Model> model,
                                       const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                                       std::size_t& n_knots)
    : OcpAbstract(model, mc_params, dt) {
  n_knots_ = n_knots;

  state_ref_.resize(n_knots_);
  for (std::size_t t = 0; t < n_knots_; ++t) {
    state_ref_[t] = state_->zero();
  }
}

LowLevelController::~LowLevelController() {}

void LowLevelController::createProblem(const SolverTypes::Type& solver_type) {
  for (int i = 0; i < n_knots_; ++i) {
    boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
        boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

    // Regularitzations
    cost_model->addCost("x_reg", cost_reg_state, 1e-6);
    cost_model->addCost("u_reg", cost_reg_control, 1e-4);  // 1e-4

    // Waypoint cost related
    crocoddyl::FramePlacement frame_ref = crocoddyl::FramePlacement(frame_base_link_id_, wp->pose);
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal =
        boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, frame_ref, actuation_->get_nu());
    cost_model->addCost("track_pos_cost", cost_goal, 1e-2);

    crocoddyl::FrameMotion vel_ref(frame_base_link_id_, *(wp->vel));
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal_vel =
        boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, vel_ref, actuation_->get_nu());
    cost_model->addCost("track_vel_cost", cost_goal_vel, 1e-2);

    // Diff & Integrated models
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_running =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_running =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_running, dt_);
  }
  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(mission_->x0_, int_models_running_, int_model_terminal_);
  setSolver(solver_type);
}

boost::shared_ptr<crocoddyl::ActionModelAbstract> LowLevelController::createDifferentialModel(){
  
}

void LowLevelController::updateStateReferenceTrajectory(const Eigen::Ref<Eigen::VectorXd>& state_new) {
  assert(state_new.size() == state_->get_nx());

  for (std::size_t t = 0; t < n_knots_ - 1; ++t) {
    state_ref_[t] = state_ref_[t + 1];
  }
  state_ref_[n_knots_ - 1] = state_new;
}

}  // namespace multicopter_mpc