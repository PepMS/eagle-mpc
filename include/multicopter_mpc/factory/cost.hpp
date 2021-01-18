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

#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/multibody/costs/state.hpp"

#include "multicopter_mpc/stage.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/factory/activation.hpp"

namespace multicopter_mpc {

class Stage;

struct CostModelTypes {
  enum Type {
    CostModelState,
    CostModelControl,
    CostModelCoMPosition,
    CostModelFramePlacement,
    CostModelFrameRotation,
    CostModelFrameTranslation,
    CostModelFrameVelocity,
    NbCostModelTypes
  };

  static std::map<std::string, Type> init_all() {
    std::map<std::string, Type> m;
    m.clear();
    m.insert({"CostModelState", CostModelState});
    m.insert({"CostModelControl", CostModelControl});
    m.insert({"CostModelCoMPosition", CostModelCoMPosition});
    m.insert({"CostModelFramePlacement", CostModelFramePlacement});
    m.insert({"CostModelFrameRotation", CostModelFrameRotation});
    m.insert({"CostModelFrameTranslation", CostModelFrameTranslation});
    m.insert({"CostModelFrameVelocity", CostModelFrameVelocity});
    std::cout << "Here in static" << std::endl;
    return m;
  }
  static const std::map<std::string, Type> all;
};

class CostModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CostModelFactory();
  ~CostModelFactory();

  boost::shared_ptr<crocoddyl::CostModelAbstract> create(const std::string& path_to_cost, const ParamsServer& server,
                                                         const boost::shared_ptr<Stage>& stage) const;

  boost::shared_ptr<ActivationModelFactory> activation_factory_;
};

}  // namespace multicopter_mpc

#endif