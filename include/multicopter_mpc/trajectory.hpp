#ifndef MULTICOPTER_MPC_TRAJECTORY_HPP_
#define MULTICOPTER_MPC_TRAJECTORY_HPP_

#include <iostream>
#include <map>
#include <string>

#include "boost/enable_shared_from_this.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/stage.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {
class Stage;
class Trajectory : public boost::enable_shared_from_this<Trajectory> {
 public:
  static boost::shared_ptr<Trajectory> create();

  void autoSetup(const ParamsServer& server);

  const boost::shared_ptr<pinocchio::Model>& get_robot_model() const;
  const boost::shared_ptr<MultiCopterBaseParams>& get_platform_params() const;
  const boost::shared_ptr<crocoddyl::StateMultibody>& get_robot_state() const;
  const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>& get_actuation() const;
  const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>& get_squash() const;
  const boost::shared_ptr<crocoddyl::ActuationSquashingModel>& get_actuation_squash() const;

 private: 
  Trajectory();

  std::map<std::string, boost::shared_ptr<Stage>> stages_;

  boost::shared_ptr<pinocchio::Model> robot_model_;

  boost::shared_ptr<MultiCopterBaseParams> platform_params_;

  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squash_;
  boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation_squash_;
};
}  // namespace multicopter_mpc

#endif