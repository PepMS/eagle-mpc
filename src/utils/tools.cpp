///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/utils/tools.hpp"

namespace eagle_mpc
{
Tools::Tools() {}

Tools::~Tools() {}

void Tools::thrustToSpeed(const Eigen::VectorXd&                          thrust,
                          const boost::shared_ptr<MultiCopterBaseParams>& params,
                          Eigen::VectorXd&                                speed)
{
    speed = (thrust.array() / params->cf_).sqrt();
}

void Tools::thrustToSpeedNormalized(const Eigen::VectorXd&                          thrust,
                                    const boost::shared_ptr<MultiCopterBaseParams>& params,
                                    Eigen::VectorXd&                                speed)
{
    speed = (thrust.array() / params->cf_).sqrt();
    speed = -1.0 + 2 * (speed.array() - params->min_prop_speed_) / (params->max_prop_speed_ - params->min_prop_speed_);
}
}  // namespace eagle_mpc