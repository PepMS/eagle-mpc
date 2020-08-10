#include "multicopter_mpc/ocp/mpc/picewise-mpc.hpp"

namespace multicopter_mpc {

PiceWiseMpc::PiceWiseMpc(const boost::shared_ptr<pinocchio::Model>& model,
                         const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
                         const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots)
    : MpcAbstract(model, mc_params, dt, mission, n_knots) {
  initializeDefaultParameters();
  mission_ = boost::make_shared<Mission>(mission->getInitialState().size());

  parameters_yaml_path_ = MULTICOPTER_MPC_OCP_DIR "/picewise-mpc.yaml";
}

PiceWiseMpc::~PiceWiseMpc() {}

std::string PiceWiseMpc::getFactoryName() { return "PiceWiseMpc"; }

boost::shared_ptr<MpcAbstract> PiceWiseMpc::createMpcController(
    const boost::shared_ptr<pinocchio::Model>& model, const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
    const double& dt, const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots) {
  return boost::make_shared<PiceWiseMpc>(model, mc_params, dt, mission, n_knots);
}

bool PiceWiseMpc::registered_ =
    FactoryMpc::get().registerMpcController(PiceWiseMpc::getFactoryName(), PiceWiseMpc::createMpcController);

void PiceWiseMpc::initializeDefaultParameters() {
  params_.w_state_position.fill(1.);
  params_.w_state_orientation.fill(1.);
  params_.w_state_velocity_lin.fill(1.);
  params_.w_state_velocity_ang.fill(1000.);

  params_.w_state_running = 1e-6;
  params_.w_control_running = 1e-4;
  params_.w_pos_running = 1e-2;
  params_.w_vel_running = 1e-2;

  params_.w_pos_terminal = 100;
  params_.w_vel_terminal = 10;
}

void PiceWiseMpc::loadParameters(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

  std::vector<std::string> state_weights = server.getParam<std::vector<std::string>>("ocp/state_weights");
  std::map<std::string, std::string> current_state =
      yaml_parser::converter<std::map<std::string, std::string>>::convert(state_weights[0]);

  params_.w_state_position = yaml_parser::converter<Eigen::Vector3d>::convert(current_state["position"]);
  params_.w_state_orientation = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["orientation"]);
  params_.w_state_velocity_lin = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_lin"]);
  params_.w_state_velocity_ang = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_ang"]);

  params_.w_state_running = server.getParam<double>("ocp/cost_running_state_weight");
  params_.w_control_running = server.getParam<double>("ocp/cost_running_control_weight");
  params_.w_pos_running = server.getParam<double>("ocp/cost_running_pos_weight");
  params_.w_vel_running = server.getParam<double>("ocp/cost_running_vel_weight");
  params_.w_pos_terminal = server.getParam<double>("ocp/cost_terminal_pos_weight");
  params_.w_vel_terminal = server.getParam<double>("ocp/cost_terminal_vel_weight");
}

void PiceWiseMpc::initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) {
  has_motion_ref_ = false;

  trajectory_generator_->createProblem(solver_type);
  trajectory_generator_->solve();

  std::size_t wp_idx = 1;
  std::size_t wp_cursor = 0;
  for (std::vector<WayPoint>::const_iterator wp = trajectory_generator_->getMission()->getWaypoints().begin();
       wp != trajectory_generator_->getMission()->getWaypoints().end(); ++wp) {
    if (wp->knots <= n_knots_) {
      mission_->addWaypoint((*wp));
      wp_cursor += wp->knots - 1;
    } else {
      std::size_t n_groups = splitWaypoint(wp->knots);
      std::size_t n_groups_big = wp->knots % n_groups;
      std::size_t n_groups_small = n_groups - n_groups_big;
      for (std::size_t i = 0; i < n_groups; ++i) {
        std::size_t wp_knots = i < n_groups_small ? wp->knots / n_groups : wp->knots / n_groups + 1;
        wp_cursor = i == 0 ? wp_cursor + wp_knots - 1 : wp_cursor + wp_knots;
        Eigen::VectorXd state_ref = trajectory_generator_->getState(wp_cursor);
        Eigen::Quaterniond quat(static_cast<Eigen::Vector4d>(state_ref.segment(3, 4)));
        if (i == n_groups - 1) {
          WayPoint wp_middle(*wp);
          wp_middle.knots = wp_knots;
          mission_->addWaypoint(wp_middle);
        } else {
          WayPoint wp_middle(wp_knots, state_ref.head(3), quat, state_ref.segment(7, 3), state_ref.segment(10, 3));
          mission_->addWaypoint(wp_middle);
        }
      }
    }
    ++wp_idx;
  }

  mission_->countTotalKnots();
  setPoseRef(0);
  setMotionRef(0);
}

std::size_t PiceWiseMpc::splitWaypoint(const std::size_t& wp_original_knots) {
  std::size_t divider = 1;
  while (wp_original_knots / double(divider) > n_knots_) {
    ++divider;
  }
  return divider;
}

void PiceWiseMpc::createProblem(const SolverTypes::Type& solver_type) {
  initializeTrajectoryGenerator(solver_type);

  for (std::size_t i = 0; i < n_knots_ - 1; ++i) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
        createRunningDifferentialModel(i);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

    diff_models_running_.push_back(diff_model);
    int_models_running_.push_back(int_model);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      createTerminalDifferentialModel(n_knots_ - 1);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);

  // Here check in unittest that terminal and last item in vector are pointing at the same place
  // check also that the size of the vectors are equal to n_knots_
  diff_models_running_.push_back(diff_model);
  int_models_running_.push_back(int_model);

  diff_model_terminal_ = diff_model;
  int_model_terminal_ = int_model;

  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(state_initial_, int_models_running_, int_model_terminal_);
  setSolver(solver_type);

  diff_model_iter_ = diff_models_running_.end() - 1;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> PiceWiseMpc::createRunningDifferentialModel(
    const std::size_t& idx_knot) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_state = createCostStateRegularization();
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control = createCostControlRegularization();

  // Regularitzations
  cost_model->addCost("state_reg", cost_state, params_.w_state_running);
  cost_model->addCost("control_reg", cost_reg_control, params_.w_control_running);

  // Waypoint cost related
  setPoseRef(idx_knot);
  setMotionRef(idx_knot);
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  cost_model->addCost("pose_desired", cost_pose, params_.w_pos_running);

  if (has_motion_ref_) {
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
        boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());
    cost_model->addCost("vel_desired", cost_vel, params_.w_vel_running);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> PiceWiseMpc::createTerminalDifferentialModel(
    const std::size_t& idx_knot) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  // Waypoint cost related
  setPoseRef(idx_knot);
  setMotionRef(idx_knot);
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  cost_model->addCost("pose_desired", cost_pose, params_.w_pos_terminal);

  if (has_motion_ref_) {
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
        boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());
    cost_model->addCost("vel_desired", cost_vel, params_.w_vel_terminal);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> PiceWiseMpc::createCostStateRegularization() {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3) = params_.w_state_position;
  state_weights.segment(3, 3) = params_.w_state_orientation;
  state_weights.segment(model_->nv, 3) = params_.w_state_velocity_lin;
  state_weights.segment(model_->nv + 3, 3) = params_.w_state_velocity_ang;

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state =
      boost::make_shared<crocoddyl::CostModelState>(state_, activation_state, state_->zero(), actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> PiceWiseMpc::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

void PiceWiseMpc::solve() {
  problem_->set_x0(state_initial_);
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iters_, false, 1e-9);
}

void PiceWiseMpc::updateProblem(const std::size_t idx_trajectory) {
  (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_running;
  if (has_motion_ref_) {
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_running;
  }

  setPoseRef(idx_trajectory);
  setMotionRef(idx_trajectory);

  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->cost);
  cost_pose->set_Mref(pose_ref_);
  if (has_motion_ref_) {
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->cost);
    cost_vel->set_vref(motion_ref_);
  }

  if (diff_model_iter_ - 1 == diff_models_running_.begin()) {
    diff_model_iter_ = diff_models_running_.end() - 1;
  } else {
    diff_model_iter_ -= 1;
  }

  // Increase the weight for the next node
  (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_terminal;
  if (has_motion_ref_) {
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_terminal;
  }
  // updateWeights(idx_trajectory);
  // updateTerminalCost(idx_trajectory);
}

void PiceWiseMpc::updateWeights(const std::size_t& idx_trajectory) {
  // Update weights
  // Decrease the current weight value of the cost (if the trajectory is not at its end)
  // if (idx_trajectory < mission_->getTotalKnots()) {
  //   (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_running;
  //   if (has_motion_ref_) {
  //     (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_running;
  //   }

  //   if (diff_model_iter_ - 1 == diff_models_running_.begin()) {
  //     diff_model_iter_ = diff_models_running_.end() - 1;
  //   } else {
  //     diff_model_iter_ -= 1;
  //   }
  // } else {
  //   if (diff_model_iter_ != diff_models_running_.begin()) {
  //     diff_model_iter_ -= 1;
  //   } else {
  //   }
  // }

  (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_running;
  if (has_motion_ref_) {
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_running;
  }

  if (diff_model_iter_ - 1 == diff_models_running_.begin()) {
    diff_model_iter_ = diff_models_running_.end() - 1;
  } else {
    diff_model_iter_ -= 1;
  }

  // Increase the weight for the next node
  (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->weight = params_.w_pos_terminal;
  if (has_motion_ref_) {
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->weight = params_.w_vel_terminal;
  }
}

void PiceWiseMpc::updateTerminalCost(const std::size_t idx_trajectory) {
  // update Cost Reference
  setPoseRef(idx_trajectory);
  setMotionRef(idx_trajectory);

  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          (*(diff_models_running_.end() - 1))->get_costs()->get_costs().find("pose_desired")->second->cost);
  cost_pose->set_Mref(pose_ref_);
  if (has_motion_ref_) {
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            (*(diff_models_running_.end() - 1))->get_costs()->get_costs().find("vel_desired")->second->cost);
    cost_vel->set_vref(motion_ref_);
  }
}

void PiceWiseMpc::setTimeStep(const double& dt) {
  dt_ = dt;
  
  std::size_t nx = mission_->getInitialState().size();
  mission_ = boost::make_shared<Mission>(nx);
  
  trajectory_generator_->setTimeStep(dt_);

  if (problem_ != nullptr) {
    diff_models_running_.clear();
    diff_model_terminal_ = nullptr;

    int_models_running_.clear();
    int_model_terminal_ = nullptr;

    problem_ = nullptr;
    solver_ = nullptr;
  }
}

const crocoddyl::FramePlacement& PiceWiseMpc::getPoseRef() const { return pose_ref_; }
const crocoddyl::FrameMotion& PiceWiseMpc::getVelocityRef() const { return motion_ref_; }
const TrajectoryGeneratorParams& PiceWiseMpc::getParams() const { return params_; };

void PiceWiseMpc::setPoseRef(const std::size_t& idx_trajectory) {
  pose_ref_.frame = frame_base_link_id_;
  pose_ref_.oMf = mission_->getWaypoints()[mission_->getWpFromTrajIdx(idx_trajectory)].pose;
}

void PiceWiseMpc::setMotionRef(const std::size_t& idx_trajectory) {
  has_motion_ref_ = mission_->getWaypoints()[mission_->getWpFromTrajIdx(idx_trajectory)].vel != boost::none;
  if (has_motion_ref_) {
    motion_ref_.frame = frame_base_link_id_;
    motion_ref_.oMf = mission_->getWaypoints()[mission_->getWpFromTrajIdx(idx_trajectory)].vel.get();
  }
}

}  // namespace multicopter_mpc
