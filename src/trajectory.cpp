#include "multicopter_mpc/trajectory.hpp"

namespace multicopter_mpc {
Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::autoSetup(const ParamsServer& server) {
  std::cout << "This is the robot's name: " << server.getParam<std::string>("robot/name") << std::endl;
  std::cout << "This is the robot's yaml_path: " << server.getParam<std::string>("robot/platform_yaml") << std::endl;

  auto stages = server.getParam<std::vector<std::map<std::string, std::string>>>("stages");
  for (auto stage: stages) {
    std::cout << "Stage name: " << stage["name"] << std::endl;
    std::cout << "Stage cost: " << stage["duration"] << std::endl;
  }
}

}  // namespace multicopter_mpc