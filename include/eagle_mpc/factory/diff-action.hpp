///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_FACTORY_DIFF_ACTION_HPP_
#define EAGLE_MPC_FACTORY_DIFF_ACTION_HPP_

#include <map>

#include "crocoddyl/core/diff-action-base.hpp"

#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"

#include "eagle_mpc/stage.hpp"

namespace eagle_mpc
{
class Stage;

enum class DifferentialActionModelTypes {
    DifferentialActionModelFreeFwdDynamics,
    DifferentialActionModelContactFwdDynamics,
    NbDifferentialActionModelTypes
};

static std::map<std::string, DifferentialActionModelTypes> DifferentialActionModelTypes_init_map()
{
    std::map<std::string, DifferentialActionModelTypes> m;
    m.clear();
    m.insert({"DifferentialActionModelFreeFwdDynamics",
              DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics});
    m.insert({"DifferentialActionModelContactFwdDynamics",
              DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics});
    return m;
}
static const std::map<std::string, DifferentialActionModelTypes> DifferentialActionModelTypes_map =
    DifferentialActionModelTypes_init_map();

class DifferentialActionModelFactory
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit DifferentialActionModelFactory();
    ~DifferentialActionModelFactory();

    boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> create(const bool&                     is_contact,
                                                                         const bool&                     squash,
                                                                         const boost::shared_ptr<Stage>& stage) const;
};

}  // namespace eagle_mpc
#endif