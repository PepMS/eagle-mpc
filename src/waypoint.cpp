#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {

WayPoint::WayPoint(int knots, Eigen::Vector3d& pos, Eigen::Quaterniond& quat)
    : knots(knots), pose(pinocchio::SE3(quat.matrix(), pos)) {}

WayPoint::WayPoint(int knots, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, Eigen::Vector3d& vel,
                   Eigen::Vector3d& rate)
    : knots(knots), pose(pinocchio::SE3(quat.matrix(), pos)), vel(pinocchio::Motion(vel, rate)) {}

}  // namespace multicopter_mpc