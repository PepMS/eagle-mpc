///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/factory/diff-action.hpp"

namespace eagle_mpc {

DifferentialActionModelFactory::DifferentialActionModelFactory() {}

DifferentialActionModelFactory::~DifferentialActionModelFactory() {}

boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> DifferentialActionModelFactory::create(
    const bool& is_contact, const bool& squash, const boost::shared_ptr<Stage>& stage) const {
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam;

  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  if (squash) {
    actuation = stage->get_trajectory()->get_actuation_squash();
  } else {
    actuation = stage->get_trajectory()->get_actuation();
  }

  if (is_contact) {
    dam = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
        stage->get_trajectory()->get_robot_state(), actuation, stage->get_contacts(), stage->get_costs(), 0, true);
  } else {
    dam = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
        stage->get_trajectory()->get_robot_state(), actuation, stage->get_costs());
  }
  return dam;
}

}  // namespace eagle_mpc
