#ifndef MULTICOPTER_MPC_WAYPOINT_HPP_
#define MULTICOPTER_MPC_WAYPOINT_HPP_

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/optional.hpp>

namespace multicopter_mpc {

struct WayPoint {
    int knots;
    pinocchio::SE3 pose;
    boost::optional<pinocchio::Motion> vel;

    WayPoint(int knots, Eigen::Vector3d& pos, Eigen::Quaterniond& quat);
    WayPoint(int knots, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, Eigen::Vector3d& vel, Eigen::Vector3d& rate);
    
    bool operator==(const WayPoint& other) {return false;}
};
}

#endif  // OPTIUAVM_ALGORITHMS_GOTO_HPP_