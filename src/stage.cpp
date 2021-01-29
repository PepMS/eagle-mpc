#include "multicopter_mpc/stage.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {
Stage::Stage(const boost::shared_ptr<Trajectory>& trajectory) : trajectory_(trajectory) {
  costs_ = boost::make_shared<crocoddyl::CostModelSum>(trajectory_->get_robot_state(),
                                                       trajectory_->get_actuation()->get_nu());
  contacts_ = boost::make_shared<crocoddyl::ContactModelMultiple>(trajectory_->get_robot_state(),
                                                                  trajectory_->get_actuation()->get_nu());
  cost_factory_ = boost::make_shared<CostModelFactory>();
  contact_factory_ = boost::make_shared<ContactModelFactory>();
  is_terminal_ = false;
}

Stage::~Stage() { std::cout << "Inside trajectory constructor" << std::endl; }

boost::shared_ptr<Stage> Stage::create(const boost::shared_ptr<Trajectory>& trajectory) {
  boost::shared_ptr<Stage> stage(new Stage(trajectory));
  return stage;
}

void Stage::autoSetup(const std::string& path_to_stages, const std::map<std::string, std::string>& stage,
                      const ParamsServer& server) {
  std::string path_to_stage = path_to_stages + stage.at("name") + "/";

  name_ = stage.at("name");
  duration_ = std::size_t(converter<int>::convert(stage.at("duration")));

  try {
    std::vector<std::string> contact_names = converter<std::vector<std::string>>::convert(stage.at("contacts"));
    for (auto contact_name : contact_names) {
      boost::shared_ptr<crocoddyl::ContactModelAbstract> contact =
          contact_factory_->create(path_to_stage + "contacts/" + contact_name + "/", server, shared_from_this());
      contacts_->addContact(contact_name, contact);

      MMPC_INFO << "Stage '" << name_ << "': added contact '" << contact_name << "'";
    }
  } catch (const std::exception& e) {
    MMPC_INFO << "Stage: " << name_ << " DOES NOT HAVE contacts";
  }

  std::vector<std::string> cost_names = converter<std::vector<std::string>>::convert(stage.at("costs"));
  for (auto cost_name : cost_names) {
    double weight = server.getParam<double>(path_to_stage + "costs/" + cost_name + "/weight");
    double active = false;
    try {
      server.getParam<double>(path_to_stage + "costs/" + cost_name + "/active");
    } catch (const std::exception& e) {
      MMPC_WARN << e.what() << " Set to true.";
      active = true;
    }
    boost::shared_ptr<crocoddyl::CostModelAbstract> cost =
        cost_factory_->create(path_to_stage + "costs/" + cost_name + "/", server, shared_from_this());
    costs_->addCost(cost_name, cost, weight, active);
    MMPC_INFO << "Stage '" << name_ << "': added cost '" << cost_name << "'";
  }
}

const boost::shared_ptr<Trajectory>& Stage::get_trajectory() const { return trajectory_; }
const boost::shared_ptr<crocoddyl::CostModelSum>& Stage::get_costs() const { return costs_; }
const boost::shared_ptr<crocoddyl::ContactModelMultiple>& Stage::get_contacts() const { return contacts_; }

const std::size_t& Stage::get_duration() const { return duration_; }

}  // namespace multicopter_mpc