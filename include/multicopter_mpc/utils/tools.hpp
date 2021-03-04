#ifndef MULTICOPTER_MPC_UTILS_TOOLS_HPP_
#define MULTICOPTER_MPC_UTILS_TOOLS_HPP_

#include <Eigen/Dense>

#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

class Tools {
 public:
  Tools();
  ~Tools();
  static void thrustToSpeed(const Eigen::VectorXd& thrust, const boost::shared_ptr<MultiCopterBaseParams>& params,
                            Eigen::VectorXd& speed);
};

}  // namespace multicopter_mpc
#endif