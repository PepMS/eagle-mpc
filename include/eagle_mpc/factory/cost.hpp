///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_FACTORY_COST_HPP_
#define EAGLE_MPC_FACTORY_COST_HPP_

#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/costs/residual.hpp"

#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/core/residuals/control.hpp"

#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/frame-velocity.hpp"
#include "crocoddyl/multibody/residuals/frame-translation.hpp"
#include "crocoddyl/multibody/residuals/frame-rotation.hpp"
#include "crocoddyl/multibody/residuals/contact-friction-cone.hpp"

#include "eagle_mpc/stage.hpp"
#include "eagle_mpc/utils/params_server.hpp"
#include "eagle_mpc/factory/activation.hpp"

namespace eagle_mpc
{
enum class CostModelTypes {
    CostModelState,
    CostModelControl,
    CostModelFramePlacement,
    CostModelFrameTranslation,
    CostModelFrameRotation,
    CostModelFrameVelocity,
    CostModelContactFrictionCone,
    NbCostModelTypes
};

static std::map<std::string, CostModelTypes> CostModelTypes_init_map()
{
    std::map<std::string, CostModelTypes> m;
    m.clear();
    m.insert({"CostModelState", CostModelTypes::CostModelState});
    m.insert({"CostModelControl", CostModelTypes::CostModelControl});
    m.insert({"CostModelFramePlacement", CostModelTypes::CostModelFramePlacement});
    m.insert({"CostModelFrameTranslation", CostModelTypes::CostModelFrameTranslation});
    m.insert({"CostModelFrameRotation", CostModelTypes::CostModelFrameRotation});
    m.insert({"CostModelFrameVelocity", CostModelTypes::CostModelFrameVelocity});
    m.insert({"CostModelContactFrictionCone", CostModelTypes::CostModelContactFrictionCone});
    return m;
}

static const std::map<std::string, CostModelTypes> CostModelTypes_map = CostModelTypes_init_map();

class Stage;
class CostModelFactory
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CostModelFactory();
    ~CostModelFactory();

    boost::shared_ptr<crocoddyl::CostModelResidual> create(const std::string&                     path_to_cost,
                                                           const boost::shared_ptr<ParamsServer>& server,
                                                           const boost::shared_ptr<crocoddyl::StateMultibody>& state,
                                                           const std::size_t&                                  nu,
                                                           CostModelTypes& cost_type) const;

    boost::shared_ptr<ActivationModelFactory> activation_factory_;
};

}  // namespace eagle_mpc

#endif