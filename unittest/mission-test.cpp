#define BOOST_TEST_MODULE mission_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/mission.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_mission_test)

BOOST_AUTO_TEST_CASE(constructors_test) {
  std::size_t nx = 5;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);

  multicopter_mpc::Mission m00(nx);

  BOOST_CHECK(x0 == m00.getInitialState());
}

BOOST_AUTO_TEST_CASE(fill_waypoints_number_waypoints) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";
  mission.fillWaypoints(mission_yaml_path);

  BOOST_CHECK(mission.getWaypoints().size() == 3);
}

BOOST_AUTO_TEST_CASE(fill_waypoints_waypoint_pose_motion) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";

  mission.fillWaypoints(mission_yaml_path);

  double time = 1.0;
  Eigen::Vector3d pos;
  pos << 0.0, 0.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d vel;
  vel << 1.0, 2.0, 3.0;
  Eigen::Vector3d rate;
  rate << -3.0, -2.0, -1.0;

  BOOST_CHECK(mission.getWaypoints()[0].vel != boost::none);
  BOOST_CHECK(time == mission.getWaypoints()[0].time);
  BOOST_CHECK(pos == mission.getWaypoints()[0].pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() ==
              Eigen::Quaterniond(mission.getWaypoints()[0].pose.rotation()).toRotationMatrix());
  BOOST_CHECK(vel == mission.getWaypoints()[0].vel->linear());
  BOOST_CHECK(rate == mission.getWaypoints()[0].vel->angular());
}

BOOST_AUTO_TEST_CASE(fill_waypoints_waypoint_pose) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";

  mission.fillWaypoints(mission_yaml_path);

  double time = 1.9;
  Eigen::Vector3d pos;
  pos << 0.0, 1.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(0, 1, 0, 0);
  BOOST_CHECK(mission.getWaypoints()[1].vel == boost::none);
  BOOST_CHECK(time == mission.getWaypoints()[1].time);
  BOOST_CHECK(pos == mission.getWaypoints()[1].pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() ==
              Eigen::Quaterniond(mission.getWaypoints()[1].pose.rotation()).toRotationMatrix());
}

BOOST_AUTO_TEST_CASE(fill_initial_state) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";

  mission.fillWaypoints(mission_yaml_path);

  Eigen::Vector3d pos;
  pos << 0.0, 2.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d vel;
  vel << -3.0, 0.0, 0.0;
  Eigen::Vector3d rate;
  rate << 0.0, 0.0, 0.0;
  BOOST_CHECK(pos == mission.getInitialState().head(3));
  BOOST_CHECK(quaternion.toRotationMatrix() ==
              Eigen::Quaterniond(mission.getInitialState()(6), mission.getInitialState()(3),
                                 mission.getInitialState()(4), mission.getInitialState()(5))
                  .toRotationMatrix());
  BOOST_CHECK(vel == mission.getInitialState().segment(7, 3));
  BOOST_CHECK(rate == mission.getInitialState().segment(10, 3));
}

BOOST_AUTO_TEST_CASE(count_total_knots) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";

  BOOST_CHECK(0 == mission.getTotalKnots());
  double dt = 0.01;
  mission.fillWaypoints(mission_yaml_path, dt);
  BOOST_CHECK(491 == mission.getTotalKnots());
  BOOST_CHECK(mission.getWpTrajIdx()[0] == 100);
  BOOST_CHECK(mission.getWpTrajIdx()[1] == 290);
  BOOST_CHECK(mission.getWpTrajIdx()[2] == 490);
}

BOOST_AUTO_TEST_CASE(get_wp_from_traj_idx) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission/mission-test.yaml";

  BOOST_CHECK(0 == mission.getTotalKnots());
  double dt = 0.01;
  mission.fillWaypoints(mission_yaml_path, dt);
  BOOST_CHECK(mission.getWpFromTrajIdx(0) == 0);
  BOOST_CHECK(mission.getWpFromTrajIdx(100) == 0);
  BOOST_CHECK(mission.getWpFromTrajIdx(101) == 1);
  BOOST_CHECK(mission.getWpFromTrajIdx(290) == 1);
  BOOST_CHECK(mission.getWpFromTrajIdx(291) == 2);
  BOOST_CHECK(mission.getWpFromTrajIdx(490) == 2);
  BOOST_CHECK(mission.getWpFromTrajIdx(491) == 2);
  BOOST_CHECK(mission.getWpFromTrajIdx(900) == 2);
}

BOOST_AUTO_TEST_SUITE_END()