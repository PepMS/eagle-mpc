#ifndef MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_
#define MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_

#include <iostream>

#include "Eigen/Dense"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"

// To be removed after the refactoring
#include "yaml_parser/params_server.hpp"
#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {
class MultiCopterBaseParams {
 public:
  MultiCopterBaseParams();
  MultiCopterBaseParams(const double& cf, const double& cm, const Eigen::MatrixXd& tau_f, const double& max_th,
                        const double& min_th, const std::string& base_link);
  // MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double min_th,
  //                       Eigen::VectorXd max_torque, Eigen::VectorXd min_torque, const std::string& base_link);
  ~MultiCopterBaseParams();

  void fill(const std::string& yaml_path);
  void autoSetup(const std::string& path_to_platform, const ParamsServer& server,
                 const boost::shared_ptr<pinocchio::Model>& robot_model);
  void autoSetup(const std::string& path_to_platform, const ParamsServer& server);
  void setControlLimits(const boost::shared_ptr<pinocchio::Model>& robot_model);

  double cf_;                   // Propeller's lift force coefficient
  double cm_;                   // Propeller's drag moment coefficient
  int n_rotors_;                // Number of rotors
  Eigen::MatrixXd tau_f_;       // From propellers thrust to body net force & torque
  double max_thrust_;           // Max thrust produced by the motor/propeller
  double min_thrust_;           // Max thrust produced by the motor/propeller
  std::string base_link_name_;  // Flying platform base_link name

  // To be used when dealing with UAM
  Eigen::VectorXd u_ub;  // Max bounds for the robot control vector
  Eigen::VectorXd u_lb;  // Min bounds for the robot control vector
  Eigen::VectorXd min_torque_;  // Max bounds for the robot control vector
  Eigen::VectorXd max_torque_;  // Min bounds for the robot control vector

  std::vector<pinocchio::SE3> rotors_pose_;
  std::vector<int> rotors_spin_dir_;
};
}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_