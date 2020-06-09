#ifndef MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_
#define MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_

#include <iostream>

#include "Eigen/Dense"

#include "yaml_parser/params_server.hpp"

namespace multicopter_mpc {
class MultiCopterBaseParams {
 public:
  MultiCopterBaseParams();
  MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double min_th,
                        const std::string& base_link);
  // MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double min_th,
  //                       Eigen::VectorXd max_torque, Eigen::VectorXd min_torque, const std::string& base_link);
  ~MultiCopterBaseParams();

  void fill(const yaml_parser::ParamsServer& server);

  double cf_;                   // Propeller's lift force coefficient
  double cm_;                   // Propeller's drag moment coefficient
  int n_rotors_;                // Number of rotors
  Eigen::MatrixXd tau_f_;       // From propellers thrust to body net force & torque
  double max_thrust_;           // Max thrust produced by the motor/propeller
  double min_thrust_;           // Max thrust produced by the motor/propeller
  std::string base_link_name_;  // Flying platform base_link name

  // To be used when dealing with UAM
  // Eigen::VectorXd max_torque_;  // Max torque for each manipulator's joint
  // Eigen::VectorXd min_torque_;  // Min torque for each manipulator's joint
};
}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_