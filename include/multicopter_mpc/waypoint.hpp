#ifndef MULTICOPTER_MPC_WAYPOINT_HPP_
#define MULTICOPTER_MPC_WAYPOINT_HPP_

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/optional.hpp>

namespace multicopter_mpc {

struct WayPoint {
  WayPoint(const double& time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat);
  WayPoint(const double& time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
           const Eigen::Vector3d& vel_linear, const Eigen::Vector3d& vel_angular);
  WayPoint(const std::size_t& knots, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
           const Eigen::Vector3d& vel_linear, const Eigen::Vector3d& vel_angular);
  bool operator==(const WayPoint& wp);
  bool operator!=(const WayPoint& wp);

  double time;
  std::size_t knots;
  pinocchio::SE3 pose;
  boost::optional<pinocchio::Motion> vel;
};
}  // namespace multicopter_mpc

#endif  // OPTIUAVM_ALGORITHMS_GOTO_HPP_