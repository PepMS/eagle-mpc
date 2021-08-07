#include "eagle_mpc/trajectory.hpp"

#include "eagle_mpc/utils/log.hpp"
namespace eagle_mpc
{
Trajectory::Trajectory()
{
    has_contact_ = false;
    dam_factory_ = boost::make_shared<DifferentialActionModelFactory>();
    iam_factory_ = boost::make_shared<IntegratedActionModelFactory>();
}

Trajectory::~Trajectory() {}

boost::shared_ptr<Trajectory> Trajectory::create()
{
    boost::shared_ptr<Trajectory> trajectory(new Trajectory());
    return trajectory;
}

void Trajectory::autoSetup(const std::string& yaml_path)
{
    ParserYaml parser(yaml_path);
    params_server_ = boost::make_shared<ParamsServer>(parser.get_params());

    std::string prefix_robot = "robot/";
    robot_model_path_        = params_server_->getParam<std::string>(prefix_robot + "urdf");

    pinocchio::Model model;
    pinocchio::urdf::buildModel(robot_model_path_, pinocchio::JointModelFreeFlyer(), model);
    robot_model_ = boost::make_shared<pinocchio::Model>(model);

    platform_params_ = boost::make_shared<MultiCopterBaseParams>();
    platform_params_->autoSetup("robot/platform/", params_server_, robot_model_);

    robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(robot_model_);
    actuation_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(robot_state_, platform_params_->tau_f_);
    squash_    = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(platform_params_->u_lb, platform_params_->u_ub,
                                                                     actuation_->get_nu());
    actuation_squash_ =
        boost::make_shared<crocoddyl::ActuationSquashingModel>(actuation_, squash_, actuation_->get_nu());

    try {
        initial_state_ = params_server_->getParam<Eigen::VectorXd>("initial_state");
    } catch (const std::exception& e) {
        initial_state_ = robot_state_->zero();
        MMPC_WARN << "Initial state not found, set to the zero state";
    }

    if (initial_state_.size() != robot_state_->get_nx()) {
        throw std::runtime_error("The specified initial state has wrong dimension. Should be " +
                                 std::to_string(robot_state_->get_nx()) + " and it has " +
                                 std::to_string(initial_state_.size()));
    }

    auto        stages_params    = params_server_->getParam<std::vector<std::map<std::string, std::string>>>("stages");
    std::size_t time             = 0;
    bool        stage_duration_0 = false;

    for (auto stage_param : stages_params) {
        boost::shared_ptr<Stage> stage = Stage::create(shared_from_this());
        stage->autoSetup("stages/", stage_param, params_server_, time);
        if (!stage_duration_0 && stage->get_duration() == 0) {
            stage_duration_0 = true;
        } else if (stage_duration_0 && stage->get_duration() == 0) {
            throw std::runtime_error(
                "Two consecutives stages cannot have duration 0. Please, unify them in a single stage.");
        } else {
            stage_duration_0 = false;
        }
        time += stage->get_duration();
        stages_.push_back(stage);
        if (!has_contact_) {
            has_contact_ = stage->get_contacts()->get_contacts().size() != 0;
        }
    }
    duration_ = time;
}

boost::shared_ptr<crocoddyl::ShootingProblem> Trajectory::createProblem(const std::size_t& dt,
                                                                        const bool&        squash,
                                                                        const std::string& integration_method) const
{
    MMPC_INFO << "Creating problem for the given stages. Contact Trajectory = " << has_contact_;
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
    boost::shared_ptr<crocoddyl::ActionModelAbstract>              terminal_model;

    bool last_duration0 = false;
    for (auto stage = stages_.begin(); stage != stages_.end(); ++stage) {
        boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam =
            dam_factory_->create(has_contact_, squash, *stage);

        boost::shared_ptr<crocoddyl::ActionModelAbstract> iam = iam_factory_->create(integration_method, dt, dam);

        std::size_t n_knots;
        if ((*stage)->get_duration() / dt == 0 && std::next(stage) != stages_.end()) {
            n_knots        = 1;
            last_duration0 = true;
        } else {
            n_knots = (*stage)->get_duration() / dt;
            if (last_duration0) {
                n_knots -= 1;
            }
            last_duration0 = false;
        }

        MMPC_INFO << "Create Problem; " << (*stage)->get_name() << ", # Knots: " << n_knots;

        iam->set_u_lb(platform_params_->u_lb);
        iam->set_u_ub(platform_params_->u_ub);

        std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> iams_stage(n_knots, iam);
        terminal_model = iam;
        running_models.insert(running_models.end(), iams_stage.begin(), iams_stage.end());
    }

    boost::shared_ptr<crocoddyl::ShootingProblem> problem =
        boost::make_shared<crocoddyl::ShootingProblem>(initial_state_, running_models, terminal_model);

    return problem;
}

void Trajectory::removeStage(const std::size_t& idx_stage)
{
    assert(idx_stage < stages_.size());

    stages_.erase(stages_.begin() + idx_stage);
}

void Trajectory::set_initial_state(const Eigen::VectorXd& initial_state)
{
    assert(initial_state.size() == robot_state_->get_nx());
    initial_state_ = initial_state;
}

const std::vector<boost::shared_ptr<Stage>>&    Trajectory::get_stages() const { return stages_; }
const boost::shared_ptr<pinocchio::Model>&      Trajectory::get_robot_model() const { return robot_model_; }
const std::string&                              Trajectory::get_robot_model_path() const { return robot_model_path_; }
const boost::shared_ptr<MultiCopterBaseParams>& Trajectory::get_platform_params() const { return platform_params_; }
const boost::shared_ptr<crocoddyl::StateMultibody>& Trajectory::get_robot_state() const { return robot_state_; }
const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>& Trajectory::get_actuation() const
{
    return actuation_;
}
const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>& Trajectory::get_squash() const { return squash_; }
const boost::shared_ptr<crocoddyl::ActuationSquashingModel>& Trajectory::get_actuation_squash() const
{
    return actuation_squash_;
}
const Eigen::VectorXd&                 Trajectory::get_initial_state() const { return initial_state_; }
const boost::shared_ptr<ParamsServer>& Trajectory::get_params_server() const { return params_server_; }
const bool&                            Trajectory::get_has_contact() const { return has_contact_; }
const std::size_t&                     Trajectory::get_duration() const { return duration_; }
}  // namespace eagle_mpc