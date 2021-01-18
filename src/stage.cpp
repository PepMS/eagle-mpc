#include "multicopter_mpc/stage.hpp"

namespace multicopter_mpc {
Stage::Stage(const boost::shared_ptr<Trajectory>& trajectory) : trajectory_(trajectory) {}

Stage::~Stage() {}

void Stage::autoSetup(const std::string& path_to_stages, const std::map<std::string, std::string>& stage,
                      const ParamsServer& server) {
  std::cout << "Stage name: " << stage.at("name") << std::endl;
  std::cout << "Stage duration: " << stage.at("duration") << std::endl;
  std::vector<std::string> costs = converter<std::vector<std::string>>::convert(stage.at("costs"));
  for (auto cost : costs) {
    std::cout << "Stage cost: " << cost << std::endl;
  }
  if (trajectory_ == nullptr) {
    std::cout << "NUUUUUL " << std::endl;


  }
}
}  // namespace multicopter_mpc