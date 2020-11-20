#include "multicopter_mpc/ocp/mpc/carrot-mpc.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

CarrotMpc::CarrotMpc(const boost::shared_ptr<pinocchio::Model>& model,
                     const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                     const boost::shared_ptr<Mission>& mission)
    : MpcAbstract(model, mc_params, mission) {
  initializeDefaultParameters();
}

CarrotMpc::~CarrotMpc() {}

std::string CarrotMpc::getFactoryName() { return "CarrotMpc"; }

boost::shared_ptr<MpcAbstract> CarrotMpc::createMpcController(
    const boost::shared_ptr<pinocchio::Model>& model, const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
    const boost::shared_ptr<Mission>& mission) {
  MMPC_INFO << "Carrot MPC controller created";
  return boost::make_shared<CarrotMpc>(model, mc_params, mission);
}

bool CarrotMpc::registered_ =
    FactoryMpc::get().registerMpcController(CarrotMpc::getFactoryName(), CarrotMpc::createMpcController);

void CarrotMpc::initializeDefaultParameters() {
  params_.weights.w_state_position.fill(1.);
  params_.weights.w_state_orientation.fill(1.);
  params_.weights.w_state_velocity_lin.fill(1.);
  params_.weights.w_state_velocity_ang.fill(1000.);

  params_.weights.w_state_running = 1e-6;
  params_.weights.w_control_running = 1e-4;
  params_.weights.w_pos_running = 1e-2;
  params_.weights.w_vel_running = 1e-2;

  params_.weights.w_pos_terminal = 100;
  params_.weights.w_vel_terminal = 10;

  params_.terminal_cost_with_wp_enabled = false;
  params_.use_wp_for_reference = false;
  params_.control_reg_trajectory = false;
}

void CarrotMpc::loadParameters(const std::string& yaml_path) {
  MpcAbstract::loadParameters(yaml_path);

  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

  std::vector<std::string> state_weights = server.getParam<std::vector<std::string>>("ocp/state_weights");
  std::map<std::string, std::string> current_state =
      yaml_parser::converter<std::map<std::string, std::string>>::convert(state_weights[0]);

  params_.weights.w_state_position = yaml_parser::converter<Eigen::Vector3d>::convert(current_state["position"]);
  params_.weights.w_state_orientation = yaml_parser::converter<Eigen::VectorXd>::convert(current_state["orientation"]);
  params_.weights.w_state_velocity_lin =
      yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_lin"]);
  params_.weights.w_state_velocity_ang =
      yaml_parser::converter<Eigen::VectorXd>::convert(current_state["velocity_ang"]);

  params_.weights.w_state_running = server.getParam<double>("ocp/cost_running_state_weight");
  params_.weights.w_control_running = server.getParam<double>("ocp/cost_running_control_weight");
  params_.weights.w_pos_running = server.getParam<double>("ocp/cost_running_pos_weight");
  params_.weights.w_vel_running = server.getParam<double>("ocp/cost_running_vel_weight");
  params_.weights.w_pos_terminal = server.getParam<double>("ocp/cost_terminal_pos_weight");
  params_.weights.w_vel_terminal = server.getParam<double>("ocp/cost_terminal_vel_weight");

  params_.terminal_cost_with_wp_enabled = server.getParam<bool>("ocp/terminal_cost_with_wp_enabled");
  params_.use_wp_for_reference = server.getParam<bool>("ocp/use_wp_for_reference");
  params_.control_reg_trajectory = server.getParam<bool>("ocp/control_reg_trajectory");

  try {
    double dt = server.getParam<double>("ocp/dt");
    setTimeStep(dt);
  } catch (const std::exception& e) {
    MMPC_WARN << "TRAJECTORY GENERATOR PARAMS. dt not found, setting default: " << dt_;
  }
}

void CarrotMpc::initializeTerminalWeights() {
  std::size_t last_wp_idx = 0;
  terminal_weights_idx_.resize(n_knots_, false);
  while (last_wp_idx < mission_->getWpTrajIdx().size() && n_knots_ - 1 >= mission_->getWpTrajIdx()[last_wp_idx]) {
    terminal_weights_idx_[mission_->getWpTrajIdx()[last_wp_idx]] = true;
    ++last_wp_idx;
  }

  // If the whole MPC horizon is longer than the mission, the tail of the horizon will be all terminal nodes
  if (last_wp_idx >= mission_->getWpTrajIdx().size()) {
    for (std::size_t i = mission_->getWpTrajIdx()[last_wp_idx - 1]; i < n_knots_; ++i) {
      terminal_weights_idx_[i] = true;
    }
  }
}

const bool CarrotMpc::existsTerminalWeight() {
  return std::find(terminal_weights_idx_.begin() + 1, terminal_weights_idx_.end(), 1) != terminal_weights_idx_.end();
}

void CarrotMpc::createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type) {
  assert(dt_ > 0.0);
  assert(n_knots_ > 0);

  has_motion_ref_ = false;
  initializeTrajectoryGenerator();
  initializeTerminalWeights();

  assert(dt_ == trajectory_generator_->getTimeStep());

  {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
        createDifferentialModel(n_knots_ - 1);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, 0.0);

    diff_model_terminal_ = diff_model;
    int_model_terminal_ = int_model;
  }

  for (int i = n_knots_ - 2; i >= 0; --i) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = createDifferentialModel(i);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_);
    diff_models_running_.insert(diff_models_running_.begin(), diff_model);
    int_models_running_.insert(int_models_running_.begin(), int_model);
  }

  // Check that the running differential model from the problem DOES NOT contain the terminal model
  problem_ = boost::make_shared<crocoddyl::ShootingProblem>(state_initial_, int_models_running_, int_model_terminal_);

  // Here check in unittest that terminal and last item in vector are pointing at the same place
  // check also that the size of the vectors are equal to n_knots_
  diff_models_running_.push_back(diff_model_terminal_);
  int_models_running_.push_back(int_model_terminal_);

  setSolver(solver_type);

  diff_model_iter_ = diff_models_running_.end() - 1;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> CarrotMpc::createDifferentialModel(
    const std::size_t& idx_knot) {
  boost::shared_ptr<crocoddyl::CostModelSum> cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state = createCostStateRegularization();

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control;
  if (params_.control_reg_trajectory) {
    cost_reg_control = createCostControlRegularization(idx_knot);
  } else {
    cost_reg_control = createCostControlRegularization();
  }

  // Regularitzations
  cost_model->addCost("state_reg", cost_reg_state, params_.weights.w_state_running);
  cost_model->addCost("control_reg", cost_reg_control, params_.weights.w_control_running);

  // Set reference if it is the last node or a WayPoint node
  if (idx_knot == n_knots_ - 1 || terminal_weights_idx_[idx_knot]) {
    setReference(idx_knot);
  }

  double w_pos = params_.weights.w_pos_running;
  double w_vel = params_.weights.w_vel_running;
  if (idx_knot == n_knots_ - 1) {
    w_pos = params_.weights.w_pos_terminal;
    w_vel = params_.weights.w_vel_terminal;
  }

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_pose =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state_, pose_ref_, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_vel =
      boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, motion_ref_, actuation_->get_nu());

  if (terminal_weights_idx_[idx_knot] || (idx_knot == n_knots_ - 1 && !existsTerminalWeight()) ||
      (idx_knot == n_knots_ - 1 && existsTerminalWeight() && params_.terminal_cost_with_wp_enabled)) {
    cost_model->addCost("pose_desired", cost_pose, w_pos, true);
    cost_model->addCost("vel_desired", cost_vel, w_vel, true);
  } else {
    cost_model->addCost("pose_desired", cost_pose, w_pos, false);
    cost_model->addCost("vel_desired", cost_vel, w_vel, false);
  }

  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);

  diff_model->set_u_lb(tau_lb_);
  diff_model->set_u_ub(tau_ub_);

  return diff_model;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CarrotMpc::createCostStateRegularization() {
  Eigen::VectorXd state_weights(state_->get_ndx());

  state_weights.head(3) = params_.weights.w_state_position;
  state_weights.segment(3, 3) = params_.weights.w_state_orientation;
  state_weights.segment(model_->nv, 3) = params_.weights.w_state_velocity_lin;
  state_weights.segment(model_->nv + 3, 3) = params_.weights.w_state_velocity_ang;

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activation_state =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_state =
      boost::make_shared<crocoddyl::CostModelState>(state_, activation_state, state_->zero(), actuation_->get_nu());

  return cost_reg_state;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CarrotMpc::createCostControlRegularization() {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, actuation_->get_nu());

  return cost_reg_control;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CarrotMpc::createCostControlRegularization(
    const std::size_t& idx_traj) {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost_reg_control =
      boost::make_shared<crocoddyl::CostModelControl>(state_, trajectory_generator_->getControl(idx_traj));

  return cost_reg_control;
}

void CarrotMpc::updateProblem(const std::size_t& idx_trajectory) {
  // first update the terminal weight vector
  std::rotate(terminal_weights_idx_.begin(), terminal_weights_idx_.begin() + 1, terminal_weights_idx_.end());
  terminal_weights_idx_.back() = std::find(mission_->getWpTrajIdx().begin(), mission_->getWpTrajIdx().end(),
                                           idx_trajectory) != mission_->getWpTrajIdx().end();

  // --> Terminal knot reference assignment <--
  if (params_.use_wp_for_reference) {
    if (terminal_weights_idx_.back() || idx_trajectory > trajectory_generator_->getKnots() - 1) {
      std::size_t wp_idx = mission_->getWaypoints().size() - 1;
      if (idx_trajectory <= trajectory_generator_->getKnots() - 1) {
        wp_idx = std::find(mission_->getWpTrajIdx().begin(), mission_->getWpTrajIdx().end(), idx_trajectory) -
                 mission_->getWpTrajIdx().begin();
      }

      pose_ref_.id = frame_base_link_id_;
      pose_ref_.placement = mission_->getWaypoints()[wp_idx].pose;
      motion_ref_.id = frame_base_link_id_;
      motion_ref_.motion = *(mission_->getWaypoints()[wp_idx].vel);
    } else {
      setReference(idx_trajectory);
    }
  } else {
    setReference(idx_trajectory);
  }

  diff_model_iter_ = diff_models_running_.end() - 1;  // This is a pointer to the terminal DAM
  // -- References
  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->cost);
  boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
      boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
          (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->cost);
  cost_pose->set_reference<crocoddyl::FramePlacement>(pose_ref_);
  cost_vel->set_reference<crocoddyl::FrameMotion>(motion_ref_);

  //  Activate or deactivate depending on the existence of a wp in the horizon
  (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->active = true;
  (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->active = true;
  if (existsTerminalWeight() && terminal_weights_idx_.back() == false && !params_.terminal_cost_with_wp_enabled) {
    (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->active = false;
    (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->active = false;
  }

  --diff_model_iter_;

  // Treat the subsequent knots
  for (std::size_t i = 0; i < n_knots_ - 1; ++i) {
    if (i == n_knots_ - 2) {
      // The knot 0 of the horizon does no need to have an active cost
    } else {
      if (terminal_weights_idx_[n_knots_ - 2 - i]) {
        // Do test for the last nodes when trajectory is over!
        if (params_.use_wp_for_reference) {
          std::size_t wp_idx =
              std::find(mission_->getWpTrajIdx().begin(), mission_->getWpTrajIdx().end(), idx_trajectory - i - 1) -
              mission_->getWpTrajIdx().begin();

          pose_ref_.id = frame_base_link_id_;
          pose_ref_.placement = mission_->getWaypoints()[wp_idx].pose;
          motion_ref_.id = frame_base_link_id_;
          motion_ref_.motion = *(mission_->getWaypoints()[wp_idx].vel);
        } else {
          setReference(idx_trajectory - i - 1);
        }
        (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->active = true;
        (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->active = true;
      } else {
        if (idx_trajectory - i - 1 > trajectory_generator_->getKnots() - 1) {
          // When the mpc controller has reached the end, the tail of the trajectory has to be active
          (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->active = true;
          (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->active = true;
        } else {
          (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->active = false;
          (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->active = false;
        }
        cost_pose = boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            (*diff_model_iter_)->get_costs()->get_costs().find("pose_desired")->second->cost);
        cost_vel = boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            (*diff_model_iter_)->get_costs()->get_costs().find("vel_desired")->second->cost);
        cost_pose->set_reference<crocoddyl::FramePlacement>(pose_ref_);
        cost_vel->set_reference<crocoddyl::FrameMotion>(motion_ref_);
      }

      --diff_model_iter_;
    }
  }
  // Increase the weight for the next node
}

void CarrotMpc::setTimeStep(const double& dt) {
  dt_ = dt;
  mission_ = nullptr;
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

void CarrotMpc::setNumberKnots(const std::size_t& n_knots) {
  n_knots_ = n_knots;

  if (problem_ != nullptr) {
    diff_models_running_.clear();
    diff_model_terminal_ = nullptr;

    int_models_running_.clear();
    int_model_terminal_ = nullptr;

    problem_ = nullptr;
    solver_ = nullptr;
  }
  terminal_weights_idx_.resize(n_knots_, false);
}

const crocoddyl::FramePlacement& CarrotMpc::getPoseRef() const { return pose_ref_; }
const crocoddyl::FrameMotion& CarrotMpc::getVelocityRef() const { return motion_ref_; }
const CarrotMpcParams& CarrotMpc::getParams() const { return params_; };

void CarrotMpc::setReference(const std::size_t& idx_trajectory) {
  state_ref_ = trajectory_generator_->getState(idx_trajectory);
  quat_ref_ = Eigen::Quaterniond(static_cast<Eigen::Vector4d>(state_ref_.segment(3, 4)));

  pose_ref_.id = frame_base_link_id_;
  pose_ref_.placement = pinocchio::SE3(quat_ref_.matrix(), static_cast<Eigen::Vector3d>(state_ref_.head(3)));
  motion_ref_.id = frame_base_link_id_;
  motion_ref_.motion = pinocchio::Motion(static_cast<Eigen::Vector3d>(state_ref_.segment(7, 3)),
                                         static_cast<Eigen::Vector3d>(state_ref_.segment(10, 3)));
}

void CarrotMpc::printInfo() { printProblem(); }

void CarrotMpc::printProblem() {
  for (std::size_t i = 0; i < n_knots_ - 1; ++i) {
    boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
        boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
            diff_models_running_[i]->get_costs()->get_costs().find("pose_desired")->second->cost);
    boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
        boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
            diff_models_running_[i]->get_costs()->get_costs().find("vel_desired")->second->cost);
    MMPC_INFO << "--------- Node " << i << " ---------";
    MMPC_INFO << "Waypoint node " << i << ": " << terminal_weights_idx_[i];
    MMPC_INFO << "Active pos cost " << i << ": "
              << diff_models_running_[i]->get_costs()->get_costs().find("pose_desired")->second->active;
    MMPC_INFO << "Active pvel cost" << i << ": "
              << diff_models_running_[i]->get_costs()->get_costs().find("vel_desired")->second->active;
    MMPC_INFO << "Position reference: \n "
              << cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation();
    MMPC_INFO << "Angular velocity reference: \n "
              << cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular();
    MMPC_INFO << "Weight pose: "
              << diff_models_running_[i]->get_costs()->get_costs().find("pose_desired")->second->weight;
    MMPC_INFO << "Weight Vel: "
              << diff_models_running_[i]->get_costs()->get_costs().find("vel_desired")->second->weight;
  }

  std::size_t i = n_knots_ - 1;
  boost::shared_ptr<crocoddyl::CostModelFramePlacement> cost_pose =
      boost::static_pointer_cast<crocoddyl::CostModelFramePlacement>(
          diff_model_terminal_->get_costs()->get_costs().find("pose_desired")->second->cost);
  boost::shared_ptr<crocoddyl::CostModelFrameVelocity> cost_vel =
      boost::static_pointer_cast<crocoddyl::CostModelFrameVelocity>(
          diff_model_terminal_->get_costs()->get_costs().find("vel_desired")->second->cost);
  MMPC_INFO << "--------- Node " << i << " ---------";
  MMPC_INFO << "Waypoint node " << i << ": " << terminal_weights_idx_[i];
  MMPC_INFO << "Active pos cost " << i << ": "
            << diff_model_terminal_->get_costs()->get_costs().find("pose_desired")->second->active;
  MMPC_INFO << "Active vel cost " << i << ": "
            << diff_model_terminal_->get_costs()->get_costs().find("vel_desired")->second->active;
  MMPC_INFO << "Position reference: \n "
            << cost_pose->get_reference<crocoddyl::FramePlacement>().placement.translation();
  MMPC_INFO << "Angular velocity reference: \n " << cost_vel->get_reference<crocoddyl::FrameMotion>().motion.angular();
  MMPC_INFO << "Weight pose: " << diff_model_terminal_->get_costs()->get_costs().find("pose_desired")->second->weight;
  MMPC_INFO << "Weight Vel: " << diff_model_terminal_->get_costs()->get_costs().find("vel_desired")->second->weight;
}
}  // namespace multicopter_mpc