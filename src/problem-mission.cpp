#include "multicopter_mpc/problem-mission.hpp"

namespace multicopter_mpc {
ProblemMission::ProblemMission(boost::shared_ptr<Mission> mission, boost::shared_ptr<MultiCopterBaseParams> mc_params,
                               boost::shared_ptr<pinocchio::Model> mc_model)
    : mission_(mission), mc_params_(mc_params), mc_model_(mc_model) {}
ProblemMission::~ProblemMission() {}

boost::shared_ptr<crocoddyl::ShootingProblem> ProblemMission::createProblem() {
  // TODO: check whether the waypoint list is full

  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(mc_model_);

  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> act_model =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state, mc_params_->n_rotors_, mc_params_->tau_f_);

  boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());

  crocoddyl::FramePlacement M_ref =
      crocoddyl::FramePlacement(mc_model_->getFrameId("base_link"), mission_->waypoints_[0].pose);
  Eigen::VectorXd state_weights = Eigen::VectorXd(mc_model_->nv * 2);
  state_weights.head(3).fill(0.1);
  state_weights.tail(mc_model_->nv * 2 - 3).fill(1000);

  boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state, M_ref, act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost = boost::make_shared<crocoddyl::CostModelState>(
      state, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights), state->zero(),
      act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> control_reg_cost =
      boost::make_shared<crocoddyl::CostModelControl>(state, act_model->get_nu());

  running_cost_model->addCost("x_reg", state_reg_cost, 1e-6);
  running_cost_model->addCost("u_reg", control_reg_cost, 1e-6);
  running_cost_model->addCost("track_pose", goal_tracking_cost, 1e-2);
  terminal_cost_model->addCost("goal_pose", goal_tracking_cost, 100);

  double dt = 3e-2;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
          boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model, running_cost_model),
          dt);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
          boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model, terminal_cost_model),
          dt);

  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models(mission_->waypoints_[0].knots,
                                                                                running_model);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(mission_->x0_, running_models, terminal_model);
  
  return problem;
}

}  // namespace multicopter_mpc