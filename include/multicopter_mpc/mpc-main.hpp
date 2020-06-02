#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#define MOTOR_TH_NORM_MIN -1.0
#define MOTOR_TH_NORM_MAX 1.0

#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

// #include "crocoddyl/core/fwd.hpp"
// #include "crocoddyl/core/solver-base.hpp"
// #include "crocoddyl/core/solvers/box-fddp.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/ocp/low-level-controller.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/path.h"
#include "multicopter_mpc/problem-mission.hpp"

namespace multicopter_mpc {

struct MultiCopterTypes {
  enum Type { Iris, Hector, NbMultiCopterTypes };
};

class MpcMain {
 public:
  MpcMain();
  MpcMain(MultiCopterTypes::Type mc_type, SolverTypes::Type solver_type);
  ~MpcMain();

 private:
  MultiCopterTypes::Type mc_type_;
  SolverTypes::Type solver_type_;

  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<MultiCopterBaseParams> mc_params_;

  double dt_;

  boost::shared_ptr<LowLevelController> low_level_controller_;
  std::size_t low_level_controller_knots_;
  boost::shared_ptr<TrajectoryGenerator> trajectory_generator_;

  // void initBoxFDDP();
  // void initSquashBoxFDDP();
  // void init();
  // void computeNormalizedControls();

  // MissionTypes::Type mission_type_;
  // std::string model_frame_name_;

  // // DDP related
  // boost::shared_ptr<crocoddyl::StateMultibody> state_;
  // boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  // boost::shared_ptr<crocoddyl::SolverAbstract> solver_;
  // boost::shared_ptr<crocoddyl::ShootingProblem> problem_opt_;

  // Eigen::VectorXd controls_normalized_;
  // Eigen::VectorXd controls_;

  // boost::shared_ptr<ProblemMission> problem_;
};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_