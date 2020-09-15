#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#define MOTOR_TH_NORM_MIN -1.0
#define MOTOR_TH_NORM_MAX 1.0

#define MOTOR_SPEED_MAX 838.0
#define MOTOR_SPEED_MIN 0.0

#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"
#include "multicopter_mpc/ocp/mpc/picewise-mpc.hpp"
#include "multicopter_mpc/ocp/mpc/rail-mpc.hpp"
#include "multicopter_mpc/path.h"

namespace multicopter_mpc {

struct MultiCopterTypes {
  enum Type { Iris, Hector, NbMultiCopterTypes };
};

class MpcMain {
 public:
  MpcMain();
  MpcMain(const MultiCopterTypes::Type& mc_type, const SolverTypes::Type& solver_type, const std::string& mission_name,
          const std::string& mpc_type, const std::string& mpc_yaml_path, const double& dt);
  ~MpcMain();

  void setCurrentState(const Eigen::Ref<Eigen::VectorXd>& current_state);
  void runMpcStep();

  void thrustToSpeed(const Eigen::Ref<const Eigen::VectorXd>& motors_thrust, Eigen::Ref<Eigen::VectorXd> motors_speed);

  const boost::shared_ptr<const MpcAbstract> getMpcController();
  const Eigen::VectorXd& getState();
  const Eigen::VectorXd& getMotorsSpeed();
  const Eigen::VectorXd& getMotorsThrust();
  const Eigen::VectorXd& getFeedForwardGains();
  const Eigen::MatrixXd& getFeedBackGains();
  void getStateDiff(const Eigen::Ref<const Eigen::VectorXd>& state0, const Eigen::Ref<const Eigen::VectorXd>& state1,
                    Eigen::Ref<Eigen::VectorXd> state_diff);

 private:
  // void computeSpeedControls();

  MultiCopterTypes::Type mc_type_;
  SolverTypes::Type solver_type_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<Mission> mission_;

  double dt_;

  boost::shared_ptr<MpcAbstract> mpc_controller_;
  std::size_t mpc_controller_knots_;

  Eigen::VectorXd state_;
  Eigen::VectorXd motor_thrust_;
  Eigen::VectorXd motor_speed_;
  Eigen::VectorXd ff_gains_;
  Eigen::MatrixXd fb_gains_;
  std::size_t trajectory_cursor_;
};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_