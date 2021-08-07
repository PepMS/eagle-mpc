#include "eagle_mpc/mpc-base.hpp"

namespace eagle_mpc
{
MpcAbstract::MpcAbstract(const std::string& yaml_path)
{
    eagle_mpc::ParserYaml parser(yaml_path);
    params_server_ = boost::make_shared<ParamsServer>(parser.get_params());

    initializeRobotObjects();
    loadParams();

    int_models_.reserve(params_.knots);
    dif_models_.reserve(params_.knots);

    cost_factory_ = boost::make_shared<CostModelFactory>();
}

void MpcAbstract::initializeRobotObjects()
{
    std::string prefix_robot = "robot/";
    robot_model_path_        = params_server_->getParam<std::string>(prefix_robot + "urdf");

    pinocchio::Model model;
    pinocchio::urdf::buildModel(robot_model_path_, pinocchio::JointModelFreeFlyer(), model);
    robot_model_ = boost::make_shared<pinocchio::Model>(model);

    platform_params_ = boost::make_shared<MultiCopterBaseParams>();
    platform_params_->autoSetup(prefix_robot + "platform/", params_server_, robot_model_);

    robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(robot_model_);
    actuation_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(robot_state_, platform_params_->tau_f_);
    squash_    = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                     actuation_->get_nu());
    actuation_squash_ =
        boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());
}

void MpcAbstract::loadParams()
{
    std::string prefix_controller  = "mpc_controller/";
    std::string integration_method = params_server_->getParam<std::string>(prefix_controller + "integration_method");
    params_.integrator_type        = IntegratedActionModelTypes_map.at(integration_method);

    params_.knots = params_server_->getParam<int>(prefix_controller + "knots");
    params_.iters = params_server_->getParam<int>(prefix_controller + "iters");
    params_.dt    = params_server_->getParam<int>(prefix_controller + "dt");

    std::string solver  = params_server_->getParam<std::string>(prefix_controller + "solver");
    params_.solver_type = SolverTypes_map.at(solver);
}

const boost::shared_ptr<pinocchio::Model>&      MpcAbstract::get_robot_model() const { return robot_model_; }
const std::string&                              MpcAbstract::get_robot_model_path() const { return robot_model_path_; }
const boost::shared_ptr<MultiCopterBaseParams>& MpcAbstract::get_platform_params() const { return platform_params_; }
const boost::shared_ptr<crocoddyl::StateMultibody>& MpcAbstract::get_robot_state() const { return robot_state_; }
const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>& MpcAbstract::get_actuation() const
{
    return actuation_;
}
const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>& MpcAbstract::get_squash() const { return squash_; }
const boost::shared_ptr<crocoddyl::ActuationSquashingModel>& MpcAbstract::get_actuation_squash() const
{
    return actuation_squash_;
}
const std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract>>& MpcAbstract::get_dif_models() const
{
    return dif_models_;
}
const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>& MpcAbstract::get_int_models() const
{
    return int_models_;
}
const boost::shared_ptr<crocoddyl::ShootingProblem>& MpcAbstract::get_problem() const { return problem_; }
const boost::shared_ptr<crocoddyl::SolverDDP>&       MpcAbstract::get_solver() const { return solver_; }

const std::size_t& MpcAbstract::get_dt() const { return params_.dt; }
const std::size_t& MpcAbstract::get_knots() const { return params_.knots; }
const std::size_t& MpcAbstract::get_iters() const { return params_.iters; }
const SolverTypes& MpcAbstract::get_solver_type() const { return params_.solver_type; }

}  // namespace eagle_mpc