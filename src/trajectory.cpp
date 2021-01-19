#include "multicopter_mpc/trajectory.hpp"

namespace multicopter_mpc {

Trajectory::Trajectory() {}

boost::shared_ptr<Trajectory> Trajectory::create() {
  boost::shared_ptr<Trajectory> trajectory(new Trajectory());
  return trajectory;
}

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
      robot_state_, platform_params_->n_rotors_, platform_params_->tau_f_);
  squash_ = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                   actuation_->get_nu());
  actuation_squash_ =
      boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());

  auto stages = server.getParam<std::vector<std::map<std::string, std::string>>>("stages");
  for (auto stage : stages) {
    boost::shared_ptr<Stage> stage_ptr = Stage::create(shared_from_this());
    stage_ptr->autoSetup("stages/", stage, server);
  }
}

const boost::shared_ptr<pinocchio::Model>& Trajectory::get_robot_model() const { return robot_model_; }
const boost::shared_ptr<MultiCopterBaseParams>& Trajectory::get_platform_params() const { return platform_params_; }
const boost::shared_ptr<crocoddyl::StateMultibody>& Trajectory::get_robot_state() const { return robot_state_; }
const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>& Trajectory::get_actuation() const {
  return actuation_;
}
const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>& Trajectory::get_squash() const { return squash_; }
const boost::shared_ptr<crocoddyl::ActuationSquashingModel>& Trajectory::get_actuation_squash() const {
  return actuation_squash_;
}

}  // namespace multicopter_mpc