#ifndef MULTICOPTER_MPC_STAGE_HPP_
#define MULTICOPTER_MPC_STAGE_HPP_

#include <map>

#include "boost/enable_shared_from_this.hpp"

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/activation-base.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

#include "multicopter_mpc/factory/cost.hpp"
namespace multicopter_mpc {
class Trajectory;
class CostModelFactory;
class Stage :public boost::enable_shared_from_this<Stage>{
 public:
  static boost::shared_ptr<Stage> create(const boost::shared_ptr<Trajectory>& trajectory);
  ~Stage();

  void autoSetup(const std::string& path_to_stages, const std::map<std::string, std::string>& stage,
                 const ParamsServer& server);

  const boost::shared_ptr<Trajectory>& get_trajectory() const;
 private:
  Stage(const boost::shared_ptr<Trajectory>& trajectory);
  boost::shared_ptr<Trajectory> trajectory_;

  boost::shared_ptr<crocoddyl::CostModelSum> costs_;
  boost::shared_ptr<CostModelFactory> cost_factory_;

  std::string name_;
  std::size_t duration_;
};

}  // namespace multicopter_mpc

#endif