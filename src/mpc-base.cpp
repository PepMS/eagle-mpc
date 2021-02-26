#include "multicopter_mpc/mpc-base.hpp"

namespace multicopter_mpc {

MpcAbstract::MpcAbstract(const boost::shared_ptr<ParamsServer>& server) {
  initializeRobotObjects(server);
  loadParams(server);

  cost_factory_ = boost::make_shared<CostModelFactory>();
}

void MpcAbstract::initializeRobotObjects(const boost::shared_ptr<ParamsServer>& server) {
  std::string prefix_robot = "robot/";
  robot_model_path_ = server->getParam<std::string>(prefix_robot + "urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(robot_model_path_, pinocchio::JointModelFreeFlyer(), model);
  robot_model_ = boost::make_shared<pinocchio::Model>(model);

  platform_params_ = boost::make_shared<MultiCopterBaseParams>();
  platform_params_->autoSetup(prefix_robot + "platform/", server, robot_model_);

  robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(robot_model_);
  actuation_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(robot_state_, platform_params_->n_rotors_,
                                                                            platform_params_->tau_f_);
  squash_ = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                   actuation_->get_nu());
  actuation_squash_ =
      boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());
}

void MpcAbstract::loadParams(const boost::shared_ptr<ParamsServer>& server) {
  std::string prefix_controller = "mpc_controller/";
  std::string integration_method = server->getParam<std::string>(prefix_controller + "integration_method");
  params_.integrator_type = IntegratedActionModelTypes_map.at(integration_method);

  params_.knots = server->getParam<int>(prefix_controller + "knots");
  params_.iters = server->getParam<int>(prefix_controller + "iters");

  std::string solver = server->getParam<std::string>(prefix_controller + "solver");
  params_.solver_type = SolverTypes_map.at(solver);
}

}  // namespace multicopter_mpc