#include "multicopter_mpc/mission.hpp"

namespace multicopter_mpc {
Mission::Mission(const std::size_t& nx) {
  x0_ = Eigen::VectorXd::Zero(nx);
  n_knots_ = 0;
}
Mission::~Mission() {}

void Mission::fillWaypoints(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_mission(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_mission.getParams());

  std::vector<std::string> waypts = server.getParam<std::vector<std::string>>("mission/waypoints");

  for (std::vector<std::string>::iterator it = waypts.begin(); it != waypts.end(); ++it) {
    std::map<std::string, std::string> waypoint =
        yaml_parser::converter<std::map<std::string, std::string>>::convert(it[0]);

    Eigen::Vector3d pos = yaml_parser::converter<Eigen::Vector3d>::convert(waypoint["pos"]);
    Eigen::VectorXd quat = yaml_parser::converter<Eigen::VectorXd>::convert(waypoint["quat"]);
    Eigen::Quaterniond quater(static_cast<Eigen::Vector4d>(quat));
    quater.normalize();
    double time = yaml_parser::converter<double>::convert(waypoint["time"]);

    bool has_velocity = true;
    Eigen::Vector3d vel_lin;
    try {
      vel_lin = yaml_parser::converter<Eigen::Vector3d>::convert(waypoint["vel_lin"]);
    } catch (std::runtime_error& e) {
      has_velocity = false;
    }

    if (has_velocity) {
      Eigen::Vector3d vel_rot = yaml_parser::converter<Eigen::Vector3d>::convert(waypoint["vel_rot"]);
      WayPoint wp(time, pos, quater, vel_lin, vel_rot);
      waypoints_.push_back(wp);
    } else {
      WayPoint wp(time, pos, quater);
      waypoints_.push_back(wp);
    }
  }

  std::vector<std::string> initial_state = server.getParam<std::vector<std::string>>("mission/initial_state");

  std::map<std::string, std::string> in_state =
      yaml_parser::converter<std::map<std::string, std::string>>::convert(initial_state[0]);

  Eigen::Vector3d pos = yaml_parser::converter<Eigen::Vector3d>::convert(in_state["pos"]);
  Eigen::VectorXd quat = yaml_parser::converter<Eigen::VectorXd>::convert(in_state["quat"]);
  Eigen::Quaterniond quater(static_cast<Eigen::Vector4d>(quat));
  quater.normalize();
  Eigen::Vector3d vel_lin = yaml_parser::converter<Eigen::Vector3d>::convert(in_state["vel_lin"]);
  Eigen::Vector3d vel_rot = yaml_parser::converter<Eigen::Vector3d>::convert(in_state["vel_rot"]);
  x0_.head(3) = pos;
  x0_(3) = quater.x();
  x0_(4) = quater.y();
  x0_(5) = quater.z();
  x0_(6) = quater.w();
  x0_.segment(7, 3) = vel_lin;
  x0_.segment(10, 3) = vel_rot;
}

void Mission::fillKnots() {
  assert(waypoints_.size() > 0);

  for (std::vector<WayPoint>::iterator wp = waypoints_.begin(); wp != waypoints_.end(); ++wp) {
    wp->knots = std::size_t(wp->time / dt_) + 1;
  }

  countTotalKnots();
}

void Mission::fillWaypoints(const std::string& yaml_path, const double& dt) {
  fillWaypoints(yaml_path);
  setTimeStep(dt);
}

void Mission::countTotalKnots() {
  size_t knots = 0;

  wp_traj_idx_.clear();
  int wp_count = 0;
  for (auto wp = waypoints_.begin(); wp != waypoints_.end(); wp++) {
    wp_count += wp->knots - 1;
    wp_traj_idx_.push_back(wp_count);

    if (wp == waypoints_.begin()) {
      knots += wp->knots;
    } else {
      knots += wp->knots - 1;
    }
  }

  n_knots_ = knots;
}

std::vector<Eigen::VectorXd> Mission::interpolateTrajectory(const std::string& interpolation_type) {
  if (interpolation_type == "SE3") {
    return interpolateSE3();
  } else if (interpolation_type == "R3SO3") {
    return interpolateR3SO3();
  } else {
    std::vector<Eigen::VectorXd> state_trajectory(n_knots_, x0_);
    return state_trajectory;
  }
}

std::vector<Eigen::VectorXd> Mission::interpolateSE3() {
  std::vector<Eigen::VectorXd> state_trajectory;
  Eigen::Quaterniond quat(static_cast<Eigen::Vector4d>(x0_.segment(3, 7)));
  Eigen::Vector3d pos(static_cast<Eigen::Vector3d>(x0_.head(3)));
  pinocchio::SE3 pose_initial(quat, pos);
  for (std::vector<WayPoint>::iterator wp = waypoints_.begin(); wp != waypoints_.end(); ++wp) {
    std::size_t knots;
    if (wp == waypoints_.begin()) {
      knots = wp->knots;
    } else {
      knots = wp->knots - 1;
    }
    double alpha = 0;
    double step = 1.0 / double(knots - 1);
    for (std::size_t i = 0; i < knots; ++i) {
      pinocchio::SE3 M = pinocchio::SE3::Interpolate(pose_initial, wp->pose, alpha);
      Eigen::VectorXd state = Eigen::VectorXd::Zero(x0_.size());
      state.head(3) = M.translation();
      quat = Eigen::Quaterniond(M.rotation());
      state(3) = quat.x();
      state(4) = quat.y();
      state(5) = quat.z();
      state(6) = quat.w();
      state_trajectory.push_back(state);
      alpha += step;
    }
    pose_initial = wp->pose;
  }

  return state_trajectory;
}

std::vector<Eigen::VectorXd> Mission::interpolateR3SO3() {
  std::vector<Eigen::VectorXd> state_trajectory;
  Eigen::Quaterniond quat(static_cast<Eigen::Vector4d>(x0_.segment(3, 7)));
  Eigen::Vector3d pos(static_cast<Eigen::Vector3d>(x0_.head(3)));
  pinocchio::SE3 pose_initial(quat, pos);
  for (std::vector<WayPoint>::iterator wp = waypoints_.begin(); wp != waypoints_.end(); ++wp) {
    std::size_t knots;
    if (wp == waypoints_.begin()) {
      knots = wp->knots;
    } else {
      knots = wp->knots - 1;
    }
    double alpha = 0;
    double step = 1.0 / double(knots - 1);
    for (std::size_t i = 0; i < knots; ++i) {
      pinocchio::SE3 M = pinocchio::SE3::Interpolate(pose_initial, wp->pose, alpha);
      Eigen::VectorXd state = Eigen::VectorXd::Zero(x0_.size());
      state.head(3) = pose_initial.translation() + alpha * (wp->pose.translation() - pose_initial.translation());
      quat = Eigen::Quaterniond(M.rotation());
      state(3) = quat.x();
      state(4) = quat.y();
      state(5) = quat.z();
      state(6) = quat.w();
      state_trajectory.push_back(state);
      alpha += step;
    }
    pose_initial = wp->pose;
  }

  return state_trajectory;
}

void Mission::setTimeStep(const double& dt) {
  dt_ = dt;

  if (waypoints_.size() > 0) {
    fillKnots();
  }
}

void Mission::setInitialState(const Eigen::VectorXd& x0) {
  assert(x0_.size() == x0.size());
  x0_ = x0;
}

void Mission::addWaypoint(WayPoint waypoint) { waypoints_.push_back(waypoint); }

const Eigen::VectorXd& Mission::getInitialState() const { return x0_; }
const std::vector<WayPoint>& Mission::getWaypoints() const { return waypoints_; }

const std::size_t& Mission::getTotalKnots() const { return n_knots_; }
const std::vector<std::size_t>& Mission::getWpTrajIdx() const { return wp_traj_idx_; }
const double& Mission::getTimeStep() const { return dt_; }

std::size_t Mission::getWpFromTrajIdx(const std::size_t& traj_idx) const {
  assert(n_knots_ > 0);
  assert(wp_traj_idx_.size() > 0);

  std::size_t idx = 0;
  while (idx < wp_traj_idx_.size() && traj_idx > wp_traj_idx_[idx]) {
    idx++;
  }

  return (idx > wp_traj_idx_.size() - 1 ? wp_traj_idx_.size() - 1 : idx);
}

}  // namespace multicopter_mpc