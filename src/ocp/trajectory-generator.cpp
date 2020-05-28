#include "multicopter_mpc/ocp/trajectory-generator.hpp"

namespace multicopter_mpc {

TrajectoryGenerator::TrajectoryGenerator(const boost::shared_ptr<pinocchio::Model> model,
                                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                         const boost::shared_ptr<Mission>& mission, const size_t& frame_base_link_id)
    : OcpAbstract(model, mc_params, frame_base_link_id), mission_(mission) {}

TrajectoryGenerator::~TrajectoryGenerator() {}

void TrajectoryGenerator::createProblem() {
  assert(mission_->waypoints_.size() > 0);

  knots_ = mission_->getTotalKnots();

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = setCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = setCostControlRegularization();

  for (std::vector<WayPoint>::const_iterator wp = mission_->waypoints_.begin(); wp != mission_->waypoints_.end();
       ++wp) {
    boost::shared_ptr<crocoddyl::CostModelSum> cost_model_running =
        boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
    boost::shared_ptr<crocoddyl::CostModelSum> cost_model_terminal =
        boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

    // Regularitzations
    cost_model_running->addCost("x_reg", cost_reg_state, 1e-6);
    cost_model_running->addCost("u_reg", cost_reg_control, 1e-2);  // 1e-4
    if (std::next(wp) != mission_->waypoints_.end()) {
      cost_model_terminal->addCost("x_reg_cost", cost_reg_state, 1e-6);
      cost_model_terminal->addCost("u_reg_cost", cost_reg_control, 1e-4);  // 1e-4
    }

    // Waypoint cost related
    crocoddyl::FramePlacement frame_ref = crocoddyl::FramePlacement(frame_base_link_id_, wp->pose);
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal =
        boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, frame_ref, actuation_->get_nu());
    cost_model_running->addCost("track_pos_cost", cost_goal, 1e-2);
    cost_model_terminal->addCost("goal_pos_cost", cost_goal, 100);

    if (wp->vel != boost::none) {
      crocoddyl::FrameMotion vel_ref(frame_base_link_id_, *(wp->vel));
      boost::shared_ptr<crocoddyl::CostModelAbstract> cost_goal_vel =
          boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, vel_ref, actuation_->get_nu());
      cost_model_running->addCost("track_vel_cost", cost_goal_vel, 1e-2);
      cost_model_terminal->addCost("goal_vel_cost", cost_goal_vel, 10);
    }

    // Diff & Integrated models
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_running =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model_running);
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model_terminal =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model_terminal);

    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_running =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_running, dt_);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model_terminal =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model_terminal, dt_);

    if (std::next(wp) != mission_->waypoints_.end()) {
      int_model_running->set_u_lb(s_lb);
      int_model_running->set_u_ub(s_ub);
      int_model_terminal->set_u_lb(s_lb);
      int_model_terminal->set_u_ub(s_ub);
      
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> int_models_running(wp->knots - 1, int_model_running);
      int_models_running.push_back(terminal_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    } else {
      running_model->set_u_lb(s_lb);
      running_model->set_u_ub(s_ub);

      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> run_models(wp->knots, running_model);
      running_models.insert(running_models.end(), run_models.begin(), run_models.end());
    }
  }
}

}  // namespace multicopter_mpc