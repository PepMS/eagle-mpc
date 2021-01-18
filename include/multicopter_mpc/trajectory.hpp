#ifndef MULTICOPTER_MPC_TRAJECTORY_HPP_
#define MULTICOPTER_MPC_TRAJECTORY_HPP_

#include <iostream>
#include <map>
#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/stage.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {
class Trajectory {
 public:
  Trajectory();
  ~Trajectory();

  void autoSetup(const ParamsServer& server);

  const boost::shared_ptr<pinocchio::Model>& get_robot_model();

 private:
  std::map<std::string, Stage> stages_;

  boost::shared_ptr<pinocchio::Model> robot_model_;

  boost::shared_ptr<MultiCopterBaseParams> platform_params_;

  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squash_;
  boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation_squash_;
};
}  // namespace multicopter_mpc

#endif