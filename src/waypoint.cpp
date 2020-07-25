#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {

WayPoint::WayPoint(const double& time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
    : time(time), pose(pinocchio::SE3(quat.matrix(), pos)) {}

WayPoint::WayPoint(const double& time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                   const Eigen::Vector3d& vel_linear, const Eigen::Vector3d& vel_angular)
    : time(time), pose(pinocchio::SE3(quat.matrix(), pos)), vel(pinocchio::Motion(vel_linear, vel_angular)) {}
    
WayPoint::WayPoint(const std::size_t& knots, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
         const Eigen::Vector3d& vel_linear, const Eigen::Vector3d& vel_angular)
    : knots(knots), pose(pinocchio::SE3(quat.matrix(), pos)), vel(pinocchio::Motion(vel_linear, vel_angular)) {}

bool WayPoint::operator==(const WayPoint& wp) {
  if (!(knots == wp.knots && time == wp.time && pose == wp.pose)) {
    return false;
  }

  if (vel && wp.vel) {
    return (wp.vel->linear() == vel->linear() && wp.vel->angular() == vel->angular());
  } else {
    return (!vel && !wp.vel);
  }
}

bool WayPoint::operator!=(const WayPoint& wp) { return !(*this == wp); }

}  // namespace multicopter_mpc