#include "eagle_mpc/stage.hpp"

#include "eagle_mpc/utils/log.hpp"

namespace eagle_mpc
{
Stage::Stage(const boost::shared_ptr<Trajectory>& trajectory) : trajectory_(trajectory)
{
    costs_           = boost::make_shared<crocoddyl::CostModelSum>(trajectory_->get_robot_state(),
                                                         trajectory_->get_actuation()->get_nu());
    contacts_        = boost::make_shared<crocoddyl::ContactModelMultiple>(trajectory_->get_robot_state(),
                                                                    trajectory_->get_actuation()->get_nu());
    cost_factory_    = boost::make_shared<CostModelFactory>();
    contact_factory_ = boost::make_shared<ContactModelFactory>();
    is_terminal_     = false;
}

Stage::~Stage() {}

boost::shared_ptr<Stage> Stage::create(const boost::shared_ptr<Trajectory>& trajectory)
{
    boost::shared_ptr<Stage> stage(new Stage(trajectory));
    return stage;
}

void Stage::autoSetup(const std::string&                        path_to_stages,
                      const std::map<std::string, std::string>& stage,
                      const boost::shared_ptr<ParamsServer>&    server,
                      std::size_t                               t_ini)
{
    std::string path_to_stage = path_to_stages + stage.at("name") + "/";

    name_          = stage.at("name");
    duration_      = std::size_t(converter<int>::convert(stage.at("duration")));
    t_ini_         = t_ini;
    is_transition_ = converter<bool>::convert(stage.at("transition"));

    try {
        std::vector<std::string> contact_names = converter<std::vector<std::string>>::convert(stage.at("contacts"));
        for (auto contact_name : contact_names) {
            ContactModelTypes                                  contact_type;
            boost::shared_ptr<crocoddyl::ContactModelAbstract> contact = contact_factory_->create(
                path_to_stage + "contacts/" + contact_name + "/", server, shared_from_this(), contact_type);
            contacts_->addContact(contact_name, contact);
            contact_types_.insert({contact_name, contact_type});
            EMPC_INFO << "Stage '" << name_ << "': added contact '" << contact_name << "'";
        }
    } catch (const std::exception& e) {
        EMPC_INFO << "Stage: " << name_ << " DOES NOT HAVE contacts";
    }

    std::vector<std::string> cost_names = converter<std::vector<std::string>>::convert(stage.at("costs"));
    for (auto cost_name : cost_names) {
        double weight = server->getParam<double>(path_to_stage + "costs/" + cost_name + "/weight");
        double active = false;
        try {
            server->getParam<double>(path_to_stage + "costs/" + cost_name + "/active");
        } catch (const std::exception& e) {
            EMPC_WARN << e.what() << " Set to true.";
            active = true;
        }
        CostModelTypes                                  cost_type;
        boost::shared_ptr<crocoddyl::CostModelAbstract> cost =
            cost_factory_->create(path_to_stage + "costs/" + cost_name + "/", server, trajectory_->get_robot_state(),
                                  trajectory_->get_actuation()->get_nu(), cost_type);
        costs_->addCost(cost_name, cost, weight, active);
        cost_types_.insert({cost_name, cost_type});

        EMPC_INFO << "Stage '" << name_ << "': added cost '" << cost_name << "'";
    }
}

void Stage::set_t_ini(const std::size_t& t_ini) { t_ini_ = t_ini; }
void Stage::set_duration(const std::size_t& duration) { duration_ = duration; }

const boost::shared_ptr<Trajectory>&                      Stage::get_trajectory() const { return trajectory_; }
const boost::shared_ptr<crocoddyl::CostModelSum>&         Stage::get_costs() const { return costs_; }
const boost::shared_ptr<crocoddyl::ContactModelMultiple>& Stage::get_contacts() const { return contacts_; }

const std::map<std::string, CostModelTypes>&    Stage::get_cost_types() const { return cost_types_; }
const std::map<std::string, ContactModelTypes>& Stage::get_contact_types() const { return contact_types_; }
const std::size_t&                              Stage::get_duration() const { return duration_; }
const std::size_t&                              Stage::get_t_ini() const { return t_ini_; }
const std::string&                              Stage::get_name() const { return name_; };
const bool&                                     Stage::get_is_terminal() const { return is_terminal_; }
const bool&                                     Stage::get_is_transition() const { return is_transition_; }

}  // namespace eagle_mpc