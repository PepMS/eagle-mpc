#include "multicopter_mpc/trajectory.hpp"

namespace multicopter_mpc {
Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::autoSetup(const ParamsServer& server) {
  std::string prefix_robot = "robot/";
  std::string model_urdf_path = server.getParam<std::string>(prefix_robot + "urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_urdf_path, pinocchio::JointModelFreeFlyer(), model);
  robot_model_ = boost::make_shared<pinocchio::Model>(model);

  platform_params_ = boost::make_shared<MultiCopterBaseParams>();
  platform_params_->autoSetup("robot/platform/", server);
  
  robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(robot_model_);
  // actuation_ =
  //     boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(robot_state_, mc_params_->n_rotors_, mc_params_->tau_f_);
      
  // auto stages = server.getParam<std::vector<std::map<std::string, std::string>>>("stages");
  // for (auto stage : stages) {
  //   std::cout << "Stage name: " << stage["name"] << std::endl;
  //   std::cout << "Stage duration: " << stage["duration"] << std::endl;
  //   std::vector<std::string> costs = converter<std::vector<std::string>>::convert(stage["costs"]);
  //   for (auto cost : costs) {
  //     std::cout << "Stage cost: " << cost << std::endl;
  //   }
  // }
}

const boost::shared_ptr<pinocchio::Model>& Trajectory::get_robot_model() { return robot_model_; }

}  // namespace multicopter_mpc