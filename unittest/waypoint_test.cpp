#define BOOST_TEST_MODULE waypoint_test test

#include <boost/test/unit_test.hpp>

// #include "multicopter_mpc/waypoint.hpp"

#include "multicopter_mpc/waypoint.hpp"

BOOST_AUTO_TEST_CASE(constructors_test) {
  int knots = 10;
  Eigen::Vector3d pos;
  pos << 1, 1, 1;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);

  multicopter_mpc::WayPoint wp00(knots, pos, quaternion);
  BOOST_CHECK(knots == wp00.knots);
  BOOST_CHECK(pos == wp00.pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(wp00.pose.rotation()).toRotationMatrix());

  Eigen::Vector3d vel;
  pos << 2, -2, 2;
  Eigen::Vector3d rate;
  pos << 3, -3, 3;
  multicopter_mpc::WayPoint wp01(knots, pos, quaternion, vel, rate);
  BOOST_CHECK(knots == wp01.knots);
  BOOST_CHECK(pos == wp01.pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(wp01.pose.rotation()).toRotationMatrix());
  BOOST_CHECK(vel == wp01.vel->linear());
  BOOST_CHECK(rate == wp01.vel->angular());
}