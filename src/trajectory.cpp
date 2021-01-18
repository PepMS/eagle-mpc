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
  platform_params_->autoSetup("robot/platform/", server, robot_model_);

  robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(robot_model_);
  actuation_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(
      robot_state_, robot_model_->nq - 6 + platform_params_->n_rotors_, platform_params_->tau_f_);
  squash_ = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                   actuation_->get_nu());
  actuation_squash_ =
      boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());

  auto stages = server.getParam<std::vector<std::map<std::string, std::string>>>("stages");
  for (auto stage : stages) {
    std::cout << "Stage name: " << stage["name"] << std::endl;
    std::cout << "Stage duration: " << stage["duration"] << std::endl;
    std::vector<std::string> costs = converter<std::vector<std::string>>::convert(stage["costs"]);
    for (auto cost : costs) {
      std::cout << "Stage cost: " << cost << std::endl;
    }
  }
}

const boost::shared_ptr<pinocchio::Model>& Trajectory::get_robot_model() { return robot_model_; }

}  // namespace multicopter_mpc