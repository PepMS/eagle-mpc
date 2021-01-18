#ifndef MULTICOPTER_MPC_STAGE_HPP_
#define MULTICOPTER_MPC_STAGE_HPP_

#include <map>

#include "crocoddyl/core/costs/cost-sum.hpp"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
namespace multicopter_mpc {
class Trajectory;
class Stage {
 public:
  Stage(const boost::shared_ptr<Trajectory>& trajectory);
  ~Stage();

  void autoSetup(const std::string& path_to_stages, const std::map<std::string, std::string>& stage,
                 const ParamsServer& server);

 private:
  // Stage();
  boost::shared_ptr<Trajectory> trajectory_;

  boost::shared_ptr<crocoddyl::CostModelSum> costs_;
};

}  // namespace multicopter_mpc

#endif