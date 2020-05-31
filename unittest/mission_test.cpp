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

  BOOST_CHECK(x0 == m00.x0_);
}

BOOST_AUTO_TEST_CASE(fill_waypoints_number_waypoints) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(_MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission.yaml";

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());
  mission.fillWaypoints(server_mission);

  BOOST_CHECK(mission.waypoints_.size() == 2);
}

BOOST_AUTO_TEST_CASE(fill_waypoints_waypoint_pose_motion) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(_MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission.yaml";

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());
  mission.fillWaypoints(server_mission);

  std::size_t knots = 100;
  Eigen::Vector3d pos;
  pos << 0.0, 0.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d vel;
  vel << 1.0, 2.0, 3.0;
  Eigen::Vector3d rate;
  rate << -3.0, -2.0, -1.0;

  BOOST_CHECK(mission.waypoints_[0].vel != boost::none);
  BOOST_CHECK(knots == mission.waypoints_[0].knots);
  BOOST_CHECK(pos == mission.waypoints_[0].pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(mission.waypoints_[0].pose.rotation()).toRotationMatrix());
  BOOST_CHECK(vel == mission.waypoints_[0].vel->linear());
  BOOST_CHECK(rate == mission.waypoints_[0].vel->angular());
}

BOOST_AUTO_TEST_CASE(fill_waypoints_waypoint_pose) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(_MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission.yaml";

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());
  mission.fillWaypoints(server_mission);

  std::size_t knots = 100;
  Eigen::Vector3d pos;
  pos << 0.0, 1.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(0, 1, 0, 0);
  BOOST_CHECK(mission.waypoints_[1].vel == boost::none);
  BOOST_CHECK(knots == mission.waypoints_[1].knots);
  BOOST_CHECK(pos == mission.waypoints_[1].pose.translation());
  BOOST_CHECK(quaternion.toRotationMatrix() == Eigen::Quaterniond(mission.waypoints_[1].pose.rotation()).toRotationMatrix());
}

BOOST_AUTO_TEST_CASE(fill_initial_state) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(_MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission.yaml";

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());
  mission.fillInitialState(server_mission);

  Eigen::Vector3d pos;
  pos << 0.0, 2.0, 2.5;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d vel;
  vel << -3.0, 0.0, 0.0;
  Eigen::Vector3d rate;
  rate << 0.0, 0.0, 0.0;
  BOOST_CHECK(pos == mission.x0_.head(3));
  BOOST_CHECK(quaternion.toRotationMatrix() ==
              Eigen::Quaterniond(mission.x0_(6), mission.x0_(3), mission.x0_(4), mission.x0_(5)).toRotationMatrix());
  BOOST_CHECK(vel == mission.x0_.segment(7, 3));
  BOOST_CHECK(rate == mission.x0_.segment(10, 3));
}

BOOST_AUTO_TEST_CASE(count_total_knots) {
  std::size_t nx = 13;
  multicopter_mpc::Mission mission(nx);

  std::string mission_yaml_path = std::string(_MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/mission.yaml";

  yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  BOOST_CHECK(0 == mission.getTotalKnots());
  mission.fillWaypoints(server_mission);
  BOOST_CHECK(200 == mission.getTotalKnots());
}

BOOST_AUTO_TEST_SUITE_END()