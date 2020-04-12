#include "multicopter_mpc/problem-mission.hpp"

namespace multicopter_mpc {
ProblemMission::ProblemMission(boost::shared_ptr<Mission> mission, boost::shared_ptr<MultiCopterBaseParams> mc_params,
                               boost::shared_ptr<pinocchio::Model> mc_model,
                               boost::shared_ptr<crocoddyl::ActuationModelAbstract> mc_actuation, const int& frame_id,
                               const double& dt)
    : mission_(mission),
      mc_params_(mc_params),
      mc_model_(mc_model),
      actuation_(mc_actuation),
      frame_id_(frame_id),
      dt_(dt) {}
ProblemMission::~ProblemMission() {}

boost::shared_ptr<crocoddyl::ShootingProblem> ProblemMission::createProblem() {
  Eigen::VectorXd s_lb = Eigen::VectorXd(mc_params_->n_rotors_);
  Eigen::VectorXd s_ub = Eigen::VectorXd(mc_params_->n_rotors_);
  s_lb.fill(mc_params_->min_thrust_);
  s_ub.fill(mc_params_->max_thrust_);

  // TODO: check whether the waypoint list is full
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model;

  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(actuation_->get_state());

  // Regularizations
  Eigen::VectorXd state_weights(mc_model_->nv * 2);
  state_weights.head(3).fill(0.1);                          // Position 1.
  state_weights.segment(3, 3).fill(1000.);                  // Orientation 1.
  state_weights.segment(mc_model_->nv, 3).fill(1000.);      // Linear velocity 1.
  state_weights.segment(mc_model_->nv + 3, 3).fill(1000.);  // Angular velocity 1000.
  boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost = boost::make_shared<crocoddyl::CostModelState>(
      state, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights), state->zero(),
      actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> control_reg_cost =
      boost::make_shared<crocoddyl::CostModelControl>(state, actuation_->get_nu());

  for (std::vector<WayPoint>::const_iterator wp = mission_->waypoints_.begin(); wp != mission_->waypoints_.end();
       ++wp) {
    boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
        boost::make_shared<crocoddyl::CostModelSum>(state, actuation_->get_nu());
    boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
        boost::make_shared<crocoddyl::CostModelSum>(state, actuation_->get_nu());

    running_cost_model->addCost("x_reg", state_reg_cost, 1e-6);
    running_cost_model->addCost("u_reg", control_reg_cost, 1e-6);  // 1e-4
    if (std::next(wp) != mission_->waypoints_.end()) {
      terminal_cost_model->addCost("x_reg_cost", state_reg_cost, 1e-6);
      terminal_cost_model->addCost("u_reg_cost", control_reg_cost, 1e-6);  // 1e-4
    }

    crocoddyl::FramePlacement M_ref = crocoddyl::FramePlacement(frame_id_, wp->pose);
    boost::shared_ptr<crocoddyl::CostModelAbstract> goal_track_pos_cost =
        boost::make_shared<crocoddyl::CostModelFramePlacement>(state, M_ref, actuation_->get_nu());
    running_cost_model->addCost("track_pos_cost", goal_track_pos_cost, 1e-2);
    terminal_cost_model->addCost("goal_pos_cost", goal_track_pos_cost, 100);

    // if (wp->vel != boost::none) {
    //   crocoddyl::FrameMotion vel_ref(frame_id_, *(wp->vel));
    //   boost::shared_ptr<crocoddyl::CostModelAbstract> goal_track_vel_cost =
    //       boost::make_shared<crocoddyl::CostModelFrameVelocity>(state, vel_ref, actuation_->get_nu());
    //   running_cost_model->addCost("track_vel_cost", goal_track_vel_cost, 1e-2);
    //   running_cost_model->addCost("goal_vel_cost", goal_track_vel_cost, 10);
    // }

    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
            boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_,
                                                                                  running_cost_model),
            dt_);
    terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_, terminal_cost_model),
        dt_);

    if (std::next(wp) != mission_->waypoints_.end()) {
      running_model->set_u_lb(s_lb);
      running_model->set_u_ub(s_ub);
      terminal_model->set_u_lb(s_lb);
      terminal_model->set_u_ub(s_ub);

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> run_models(wp->knots - 1, running_model);
      run_models.push_back(terminal_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    } else {
      running_model->set_u_lb(s_lb);
      running_model->set_u_ub(s_ub);
      
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> run_models(wp->knots, running_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    }
  }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(mission_->x0_, running_models, terminal_model);

  return problem;
}

}  // namespace multicopter_mpc