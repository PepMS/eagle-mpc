///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "multicopter_mpc/utils/tools.hpp"

namespace multicopter_mpc {

Tools::Tools(){}

Tools::~Tools(){}

void Tools::thrustToSpeed(const Eigen::VectorXd& thrust, const boost::shared_ptr<MultiCopterBaseParams>& params,
                          Eigen::VectorXd& speed) {
  speed = (thrust.array() / params->cf_).sqrt();
}
}  // namespace multicopter_mpc