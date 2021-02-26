#ifndef MULTICOPTER_MPC_STAGE_HPP_
#define MULTICOPTER_MPC_STAGE_HPP_

#include <map>

#include "boost/enable_shared_from_this.hpp"

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/activation-base.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

#include "multicopter_mpc/factory/cost.hpp"
#include "multicopter_mpc/factory/diff-action.hpp"
#include "multicopter_mpc/factory/contacts.hpp"

namespace multicopter_mpc {
class Trajectory;
class CostModelFactory;
class ContactModelFactory;
enum class CostModelTypes;
enum class ContactModelTypes;

class Stage : public boost::enable_shared_from_this<Stage> {
 public:
  static boost::shared_ptr<Stage> create(const boost::shared_ptr<Trajectory>& trajectory);
  ~Stage();

  void autoSetup(const std::string& path_to_stages, const std::map<std::string, std::string>& stage,
                 const boost::shared_ptr<ParamsServer>& server);

  const boost::shared_ptr<Trajectory>& get_trajectory() const;
  const boost::shared_ptr<crocoddyl::CostModelSum>& get_costs() const;
  const boost::shared_ptr<crocoddyl::ContactModelMultiple>& get_contacts() const;

  const std::map<std::string, CostModelTypes>& get_cost_types() const;
  const std::map<std::string, ContactModelTypes>& get_contact_types() const;
  const std::size_t& get_duration() const;
  const bool& get_is_transition() const;
  const bool& get_is_terminal() const;
  const std::string& get_name() const;

 private:
  Stage(const boost::shared_ptr<Trajectory>& trajectory);

  boost::shared_ptr<crocoddyl::CostModelSum> costs_;
  boost::shared_ptr<crocoddyl::ContactModelMultiple> contacts_;
  boost::shared_ptr<Trajectory> trajectory_;

  std::map<std::string, CostModelTypes> cost_types_;
  std::map<std::string, ContactModelTypes> contact_types_;

  boost::shared_ptr<CostModelFactory> cost_factory_;
  boost::shared_ptr<ContactModelFactory> contact_factory_;

  std::string name_;
  std::size_t duration_;

  bool is_terminal_;
  bool is_transition_;
};

}  // namespace multicopter_mpc

#endif