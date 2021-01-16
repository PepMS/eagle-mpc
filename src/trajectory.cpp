#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/parser_yaml.hpp"

namespace multicopter_mpc {
Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::loadStagesFromYaml(const std::string& yaml_path) {
  multicopter_mpc::ParserYaml yaml_mission(yaml_path);
}
}  // namespace multicopter_mpc