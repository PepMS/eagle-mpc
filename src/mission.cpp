#include "multicopter_mpc/mission.hpp"

namespace multicopter_mpc {
Mission::Mission(const std::size_t& nx) {
  x0_ = Eigen::VectorXd::Zero(nx);
  n_knots_ = 0;
}
Mission::~Mission() {}

void Mission::fillWaypoints(const yaml_parser::ParamsServer& server) {
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
}

void Mission::fillWaypointsKnots(const double& dt) {
  assert(waypoints_.size() > 0);

  for (std::vector<WayPoint>::iterator wp = waypoints_.begin(); wp != waypoints_.end(); ++wp) {
    wp->knots = std::size_t(wp->time / dt) + 1;
  }

  countTotalKnots();
}

void Mission::fillWaypoints(const yaml_parser::ParamsServer& server, const double& dt) {
  fillWaypoints(server);
  fillWaypointsKnots(dt);
}

// This way of adding waypoints will probably be done in the controller side
void Mission::fillWaypoints(const std::vector<Eigen::VectorXd>& state_trajectory, const std::size_t& llc_knots) {
  std::size_t cursor = 0;
  std::size_t num_wp = state_trajectory.size() / (llc_knots - 1);

  for (std::size_t i = 0; i < num_wp; ++i) {
    cursor += llc_knots - 1;

    // To be removed: hardcoded indicies
    Eigen::Vector3d pos = state_trajectory[cursor].head(3);
    Eigen::Quaterniond quat(static_cast<Eigen::Vector4d>(state_trajectory[cursor].segment(3, 4)));
    Eigen::Vector3d vel_lin = state_trajectory[cursor].segment(7, 3);
    Eigen::Vector3d vel_rot = state_trajectory[cursor].segment(10, 3);

    WayPoint wp(llc_knots, pos, quat, vel_lin, vel_rot);
    waypoints_.push_back(wp);
  }

  countTotalKnots();
  x0_ = state_trajectory[0];
}

void Mission::fillInitialState(const yaml_parser::ParamsServer& server) {
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

void Mission::setInitialState(const Eigen::VectorXd& x0) {
  assert(x0_.size() == x0.size());
  x0_ = x0;
}

const Eigen::VectorXd& Mission::getInitialState() const { return x0_; }
const std::vector<WayPoint>& Mission::getWaypoints() const { return waypoints_; }

const std::size_t& Mission::getTotalKnots() const { return n_knots_; }
const std::vector<std::size_t>& Mission::getWpTrajIdx() const { return wp_traj_idx_; }

std::size_t Mission::getWpFromTrajIdx(const std::size_t& traj_idx) const {
  assert(n_knots_ > 0);
  std::size_t idx = 0;
  while (idx < wp_traj_idx_.size() && traj_idx > wp_traj_idx_[idx]) {
    idx++;
  }

  return (idx > wp_traj_idx_.size() - 1 ? wp_traj_idx_.size() - 1 : idx);
}

}  // namespace multicopter_mpc