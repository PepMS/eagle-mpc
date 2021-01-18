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

#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

struct ActivationModelTypes {
  enum Type {
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

  static std::map<std::string, Type> init_all() {
    std::map<std::string, Type> m;
    m.clear();
    m.insert({"ActivationModelQuad", ActivationModelQuad});
    m.insert({"ActivationModelQuadFlatExp", ActivationModelQuadFlatExp});
    m.insert({"ActivationModelQuadFlatLog", ActivationModelQuadFlatLog});
    m.insert({"ActivationModelSmooth1Norm", ActivationModelSmooth1Norm});
    m.insert({"ActivationModelSmooth2Norm", ActivationModelSmooth2Norm});
    m.insert({"ActivationModelWeightedQuad", ActivationModelWeightedQuad});
    m.insert({"ActivationModelQuadraticBarrier", ActivationModelQuadraticBarrier});
    m.insert({"ActivationModelWeightedQuadraticBarrier", ActivationModelWeightedQuadraticBarrier});
    return m;
  }
  static const std::map<std::string, Type> all;
};

class ActivationModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ActivationModelFactory();
  ~ActivationModelFactory();

  boost::shared_ptr<crocoddyl::ActivationModelAbstract> create(const std::string& path_to_cost,
                                                               const ParamsServer& server,
                                                               const std::size_t& nr) const;
};

}  // namespace multicopter_mpc

#endif