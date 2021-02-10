///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_FACTORY_COST_HPP_
#define MULTICOPTER_MPC_FACTORY_COST_HPP_

#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/costs/control.hpp"

#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/frame-translation.hpp"
#include "crocoddyl/multibody/costs/contact-friction-cone.hpp"

#include "multicopter_mpc/stage.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/factory/activation.hpp"

namespace multicopter_mpc {

enum class CostModelTypes {
  CostModelState,
  CostModelControl,
  CostModelFramePlacement,
  CostModelFrameTranslation,
  CostModelFrameVelocity,
  CostModelContactFrictionCone,
  NbCostModelTypes
};

static std::map<std::string, CostModelTypes> CostModelTypes_init_map() {
  std::map<std::string, CostModelTypes> m;
  m.clear();
  m.insert({"CostModelState", CostModelTypes::CostModelState});
  m.insert({"CostModelControl", CostModelTypes::CostModelControl});
  m.insert({"CostModelFramePlacement", CostModelTypes::CostModelFramePlacement});
  m.insert({"CostModelFrameTranslation", CostModelTypes::CostModelFrameTranslation});
  m.insert({"CostModelFrameVelocity", CostModelTypes::CostModelFrameVelocity});
  m.insert({"CostModelContactFrictionCone", CostModelTypes::CostModelContactFrictionCone});
  return m;
}

static const std::map<std::string, CostModelTypes> CostModelTypes_map = CostModelTypes_init_map();

class Stage;
class CostModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CostModelFactory();
  ~CostModelFactory();

  boost::shared_ptr<crocoddyl::CostModelAbstract> create(const std::string& path_to_cost, const ParamsServer& server,
                                                         const boost::shared_ptr<Stage>& stage,
                                                         CostModelTypes& cost_type) const;

  boost::shared_ptr<ActivationModelFactory> activation_factory_;
};

}  // namespace multicopter_mpc

#endif