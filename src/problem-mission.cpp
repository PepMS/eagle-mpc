#include "multicopter_mpc/problem-mission.hpp"

namespace multicopter_mpc {
ProblemMission::ProblemMission(boost::shared_ptr<Mission> mission, boost::shared_ptr<MultiCopterBaseParams> mc_params,
                               boost::shared_ptr<pinocchio::Model> mc_model, const int& frame_id, const double& dt)
    : mission_(mission), mc_params_(mc_params), mc_model_(mc_model), frame_id_(frame_id), dt_(dt) {}
ProblemMission::~ProblemMission() {}

boost::shared_ptr<crocoddyl::ShootingProblem> ProblemMission::createProblem() {
  // TODO: check whether the waypoint list is full
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model;

  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(mc_model_);

  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> act_model =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state, mc_params_->n_rotors_, mc_params_->tau_f_);

  // Regularizations
  Eigen::VectorXd state_weights(mc_model_->nv * 2);
  state_weights.head(3).fill(1.);                           // Position
  state_weights.segment(3, 3).fill(1.);                     // Orientation
  state_weights.segment(mc_model_->nv, 3).fill(1.);         // Linear velocity
  state_weights.segment(mc_model_->nv + 3, 3).fill(1000.);  // Angular velocity
  boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost = boost::make_shared<crocoddyl::CostModelState>(
      state, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights), state->zero(),
      act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> control_reg_cost =
      boost::make_shared<crocoddyl::CostModelControl>(state, act_model->get_nu());

  for (std::vector<WayPoint>::const_iterator wp = mission_->waypoints_.begin(); wp != mission_->waypoints_.end();
       ++wp) {
    boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
        boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());
    boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
        boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());

    running_cost_model->addCost("x_reg", state_reg_cost, 1e-6);
    running_cost_model->addCost("u_reg", control_reg_cost, 1e-4);
    if (std::next(wp) != mission_->waypoints_.end()) {
      terminal_cost_model->addCost("x_reg_cost", state_reg_cost, 1e-6);
      terminal_cost_model->addCost("u_reg_cost", control_reg_cost, 1e-4);
    }

    crocoddyl::FramePlacement M_ref = crocoddyl::FramePlacement(frame_id_, wp->pose);
    boost::shared_ptr<crocoddyl::CostModelAbstract> goal_track_pos_cost =
        boost::make_shared<crocoddyl::CostModelFramePlacement>(state, M_ref, act_model->get_nu());
    running_cost_model->addCost("track_pos_cost", goal_track_pos_cost, 1e-2);
    terminal_cost_model->addCost("goal_pos_cost", goal_track_pos_cost, 100);

    if (wp->vel != boost::none) {
      crocoddyl::FrameMotion vel_ref(frame_id_, *(wp->vel));
      boost::shared_ptr<crocoddyl::CostModelAbstract> goal_track_vel_cost =
          boost::make_shared<crocoddyl::CostModelFrameVelocity>(state, vel_ref, act_model->get_nu());
      running_cost_model->addCost("track_vel_cost", goal_track_vel_cost, 1e-2);
      running_cost_model->addCost("goal_vel_cost", goal_track_vel_cost, 10);
    }

    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
            boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model,
                                                                                  running_cost_model),
            dt_);
    terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model, terminal_cost_model),
        dt_);

    if (std::next(wp) != mission_->waypoints_.end()) {
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> run_models(wp->knots - 1, running_model);
      run_models.push_back(terminal_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    } else {
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> run_models(wp->knots, running_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    }
  }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(mission_->x0_, running_models, terminal_model);

  return problem;
}

}  // namespace multicopter_mpc