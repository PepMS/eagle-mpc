///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_FACTORY_ACTIVATION_COST_HPP_
#define MULTICOPTER_MPC_FACTORY_ACTIVATION_COST_HPP_

#include <vector>
#include <string>
#include <map>

#include "crocoddyl/core/activation-base.hpp"
#include "crocoddyl/core/activations/quadratic.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic-barrier.hpp"

#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

enum class ActivationModelTypes {
  ActivationModelQuad,
  ActivationModelQuadFlatExp,
  ActivationModelQuadFlatLog,
  ActivationModelSmooth1Norm,
  ActivationModelSmooth2Norm,
  ActivationModelWeightedQuad,
  ActivationModelQuadraticBarrier,
  ActivationModelWeightedQuadraticBarrier,
  NbActivationModelTypes
};

static std::map<std::string, ActivationModelTypes> ActivationModelTypes_init_map() {
  std::map<std::string, ActivationModelTypes> m;
  m.clear();
  m.insert({"ActivationModelQuad", ActivationModelTypes::ActivationModelQuad});
  m.insert({"ActivationModelQuadFlatExp", ActivationModelTypes::ActivationModelQuadFlatExp});
  m.insert({"ActivationModelQuadFlatLog", ActivationModelTypes::ActivationModelQuadFlatLog});
  m.insert({"ActivationModelSmooth1Norm", ActivationModelTypes::ActivationModelSmooth1Norm});
  m.insert({"ActivationModelSmooth2Norm", ActivationModelTypes::ActivationModelSmooth2Norm});
  m.insert({"ActivationModelWeightedQuad", ActivationModelTypes::ActivationModelWeightedQuad});
  m.insert({"ActivationModelQuadraticBarrier", ActivationModelTypes::ActivationModelQuadraticBarrier});
  m.insert({"ActivationModelWeightedQuadraticBarrier", ActivationModelTypes::ActivationModelWeightedQuadraticBarrier});
  return m;
}

static const std::map<std::string, ActivationModelTypes> ActivationModelTypes_map = ActivationModelTypes_init_map();

class ActivationModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ActivationModelFactory();
  ~ActivationModelFactory();

  boost::shared_ptr<crocoddyl::ActivationModelAbstract> create(const std::string& path_to_cost,
                                                               const boost::shared_ptr<ParamsServer>& server,
                                                               const std::size_t& nr) const;
};

}  // namespace multicopter_mpc

#endif