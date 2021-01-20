#include "multicopter_mpc/trajectory.hpp"

namespace multicopter_mpc {

Trajectory::Trajectory() {
  has_contact_ = false;
  dam_factory_ = boost::make_shared<DifferentialActionModelFactory>();
  iam_factory_ = boost::make_shared<IntegratedActionModelFactory>();
}

Trajectory::~Trajectory() { std::cout << "Inside trajectory constructor" << std::endl; }

boost::shared_ptr<Trajectory> Trajectory::create() {
  boost::shared_ptr<Trajectory> trajectory(new Trajectory());
  // boost::shared_ptr<Trajectory> trajectory = boost::make_shared<Trajectory>();
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
  actuation_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(robot_state_, platform_params_->n_rotors_,
                                                                            platform_params_->tau_f_);
  squash_ = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                   actuation_->get_nu());
  actuation_squash_ =
      boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());

  auto stages = server.getParam<std::vector<std::map<std::string, std::string>>>("stages");
  for (auto stage : stages) {
    boost::shared_ptr<Stage> stage_ptr = Stage::create(shared_from_this());
    stage_ptr->autoSetup("stages/", stage, server);
    stages_.push_back(stage_ptr);
    if (!has_contact_ && stage_ptr->get_contacts()->get_contacts().size() > 0) {
      has_contact_ = true;
    }
  }
}

boost::shared_ptr<crocoddyl::ShootingProblem> Trajectory::createProblem(const std::size_t& dt, const bool& squash,
                                                                        const Eigen::VectorXd& x0,
                                                                        const std::string& integration_method) const {
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model;

  for (auto stage = stages_.begin(); stage != stages_.end(); ++stage) {
    boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam =
        dam_factory_->create(has_contact_, squash, *stage);

    boost::shared_ptr<crocoddyl::ActionModelAbstract> iam = iam_factory_->create(integration_method, dt, dam);

    std::size_t n_knots = (*stage)->get_duration() / dt + 1;
    if (std::next(stage) == stages_.end()) {
      n_knots -= 1;
      terminal_model = iam_factory_->create(integration_method, 0, dam);
    }

    iam->set_u_lb(platform_params_->u_lb);
    iam->set_u_ub(platform_params_->u_ub);

    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> iams_stage(n_knots, iam);
    // terminal_model = iam;
    running_models.insert(running_models.begin(), iams_stage.begin(), iams_stage.end());
    std::cout << "Running models at stage " << (*stage)->get_duration() << ": " << running_models.size() << std::endl;
  }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);

  return problem;
}

const std::vector<boost::shared_ptr<Stage>>& Trajectory::get_stages() const { return stages_; }
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