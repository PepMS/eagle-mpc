///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_UTILS_TOOLS_HPP_
#define EAGLE_MPC_UTILS_TOOLS_HPP_

#include <Eigen/Dense>

#include "eagle_mpc/multicopter-base-params.hpp"

namespace eagle_mpc
{
class Tools
{
    public:
    Tools();
    ~Tools();
    static void thrustToSpeed(const Eigen::VectorXd&                          thrust,
                              const boost::shared_ptr<MultiCopterBaseParams>& params,
                              Eigen::VectorXd&                                speed);

    static void thrustToSpeedNormalized(const Eigen::VectorXd&                          thrust,
                                        const boost::shared_ptr<MultiCopterBaseParams>& params,
                                        Eigen::VectorXd&                                speed);
};

}  // namespace eagle_mpc
#endif