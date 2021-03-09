#include "multicopter_mpc/utils/tools.hpp"

namespace multicopter_mpc {

Tools::Tools(){}

Tools::~Tools(){}

void Tools::thrustToSpeed(const Eigen::VectorXd& thrust, const boost::shared_ptr<MultiCopterBaseParams>& params,
                          Eigen::VectorXd& speed) {
  speed = (thrust.array() / params->cf_).sqrt();
}
}  // namespace multicopter_mpc