#define BOOST_TEST_MODULE waypoint_test test

#include <boost/test/unit_test.hpp>

#include "eagle_mpc/waypoint.hpp"

BOOST_AUTO_TEST_SUITE(eagle_mpc_waypoint_test)

BOOST_AUTO_TEST_CASE(constructors_test) {
  double time = 1.0;
  Eigen::Vector3d pos;
  pos << 1, 1, 1;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);

  eagle_mpc::WayPoint wp00(time, pos, quaternion);
  BOOST_CHECK(wp00.vel == boost::none);
  BOOST_CHECK(time == wp00.time);
  BOOST_CHECK(pos == wp00.pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(wp00.pose.rotation()).toRotationMatrix());

  Eigen::Vector3d vel;
  vel << 2, -2, 2;
  Eigen::Vector3d rate;
  rate << 3, -3, 3;
  eagle_mpc::WayPoint wp01(time, pos, quaternion, vel, rate);
  BOOST_CHECK(wp01.vel != boost::none);
  BOOST_CHECK(time == wp01.time);
  BOOST_CHECK(pos == wp01.pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(wp01.pose.rotation()).toRotationMatrix());
  BOOST_CHECK(vel == wp01.vel->linear());
  BOOST_CHECK(rate == wp01.vel->angular());
}

BOOST_AUTO_TEST_CASE(equal_operator_test) {
  std::size_t knots = 10;
  Eigen::Vector3d pos;
  pos << 1, 1, 1;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d vel;
  vel << 2, -2, 2;
  Eigen::Vector3d rate;
  rate << 3, -3, 3;

  eagle_mpc::WayPoint wp_wo_vel_1(knots, pos, quaternion);
  eagle_mpc::WayPoint wp_wo_vel_2(wp_wo_vel_1);
  wp_wo_vel_2.pose.translation()(2) = 0;

  eagle_mpc::WayPoint wp_vel_1(knots, pos, quaternion, vel, rate);
  eagle_mpc::WayPoint wp_vel_2(knots, pos, quaternion, vel, rate);
  wp_vel_2.vel->linear()(2) = 0;
  
  BOOST_CHECK(wp_wo_vel_1 == wp_wo_vel_1);
  BOOST_CHECK(wp_wo_vel_1 != wp_wo_vel_2);
  BOOST_CHECK(wp_wo_vel_1 != wp_vel_1);
  BOOST_CHECK(wp_vel_1 == wp_vel_1);
  BOOST_CHECK(wp_vel_1 != wp_vel_2);
}

BOOST_AUTO_TEST_SUITE_END()