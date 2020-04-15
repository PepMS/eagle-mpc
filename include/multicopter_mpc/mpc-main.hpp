#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#include <string>

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/solver-base.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"

#include "yaml_parser/parser_yaml.hpp"
#include "yaml_parser/params_server.hpp"

#include "multicopter_mpc/problem-mission.hpp"

namespace multicopter_mpc {

struct MultiCopterTypes {
  enum Type { Iris, Hector, NbMultiCopterTypes };
};

struct MissionTypes {
  enum Type { Hover, TakeOff, Passthrough, NbMissionTypes };
};

struct SolverTypes {
  enum Type { BoxFDDP, SquashBoxFDDP, NbSolverTypes };
};

class MpcMain {
 public:
  MpcMain(MultiCopterTypes mc_type, MissionTypes mission_type, SolverTypes solver_type);
  ~MpcMain();

 private:
  void initBoxFDDP();
  void initSquashBoxFDDP();
  void init();

  MultiCopterTypes mc_type_;
  MissionTypes mission_type_;

  // DDP related
  pinocchio::Model model_;
  crocoddyl::StateMultibody state_;
  MultiCopterBaseParams params_;
  Mission mission_;
  crocoddyl::ActuationModelMultiCopterBase actuation_;

  SolverTypes solver_type_;
  crocoddyl::SolverBoxFDDP solver_BoxFDDP_;
  crocoddyl::ShootingProblem problem_;
};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_