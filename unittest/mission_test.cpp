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

bool checkWaypoint(const multicopter_mpc::WayPoint& wp, const std::size_t& knots,
                   const Eigen::Ref<Eigen::Vector3d>& pos, const Eigen::Quaterniond& quat,
                   const Eigen::Ref<Eigen::Vector3d>& vel, const Eigen::Ref<Eigen::Vector3d>& rate) {
  bool is_equal = false;
  if (wp.vel != boost::none) {
    is_equal = knots == wp.knots && pos == wp.pose.translation() &&
               quat.toRotationMatrix() == Eigen::Quaterniond(wp.pose.rotation()).toRotationMatrix() &&
               vel == wp.vel->linear() && rate == wp.vel->angular();
  } else {
    is_equal = knots == wp.knots && pos == wp.pose.translation() &&
               quat.toRotationMatrix() == Eigen::Quaterniond(wp.pose.rotation()).toRotationMatrix();
  }

  return is_equal;
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

  BOOST_CHECK(checkWaypoint(mission.waypoints_[0], knots, pos, quaternion, vel, rate));
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
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  BOOST_CHECK(checkWaypoint(mission.waypoints_[1], knots, pos, quaternion, zero, zero));
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

BOOST_AUTO_TEST_SUITE_END()