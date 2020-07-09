#ifndef MULTICOPTER_MPC_MISSION_HPP_
#define MULTICOPTER_MPC_MISSION_HPP_

#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {
class Mission {
 public:
  Mission(const std::size_t& nx);
  ~Mission();

  void fillWaypoints(const yaml_parser::ParamsServer& server);
  void fillWaypoints(const yaml_parser::ParamsServer& server, const double& dt);
  void fillWaypoints(const std::vector<Eigen::VectorXd>& state_trajectory, const std::size_t& llc_knots);
  void fillWaypointsKnots(const double& dt);
  void fillInitialState(const yaml_parser::ParamsServer& server);

  void setInitialState(const Eigen::VectorXd& x0);
  const Eigen::VectorXd& getInitialState() const;
  const std::vector<WayPoint>& getWaypoints() const;
  const std::size_t& getTotalKnots() const;
  const std::vector<std::size_t>& getWpTrajIdx() const;
  
  std::size_t getWpFromTrajIdx(const std::size_t& traj_idx) const;

 private:
  void countTotalKnots();
  
  std::vector<WayPoint> waypoints_;
  Eigen::VectorXd x0_;

  std::size_t n_knots_;
  std::vector<std::size_t> wp_traj_idx_;
};
}  // namespace multicopter_mpc

#endif  // MULTICOPTER_MPC_MISSION_HPP_