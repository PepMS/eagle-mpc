///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/factory/int-action.hpp"

namespace eagle_mpc
{
IntegratedActionModelFactory::IntegratedActionModelFactory() {}

IntegratedActionModelFactory::~IntegratedActionModelFactory() {}

boost::shared_ptr<crocoddyl::ActionModelAbstract> IntegratedActionModelFactory::create(
    const std::string&                                                   integration_method,
    const std::size_t&                                                   dt,
    const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract>& diff_model) const
{
    boost::shared_ptr<crocoddyl::ActionModelAbstract> iam;
    double                                            dt_s = double(dt) / 1000.;

    switch (IntegratedActionModelTypes_map.at(integration_method)) {
        case IntegratedActionModelTypes::IntegratedActionModelEuler:
            iam = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt_s);
            break;

        case IntegratedActionModelTypes::IntegratedActionModelRK4:
            iam = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(diff_model, dt_s);
            break;

        default:
            throw std::runtime_error("The integration method " + integration_method + " does not exists.");
            break;
    }
    return iam;
}

}  // namespace eagle_mpc
