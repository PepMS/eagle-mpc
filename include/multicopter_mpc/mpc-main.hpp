#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

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
  MpcMain(MultiCopterTypes::Type mc_type, MissionTypes::Type mission_type, SolverTypes::Type solver_type);
  ~MpcMain();

 private:
  void initBoxFDDP();
  void initSquashBoxFDDP();
  void init();

  MultiCopterTypes::Type mc_type_;
  MissionTypes::Type mission_type_;

  // DDP related
  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<MultiCopterBaseParams> params_;
  boost::shared_ptr<Mission> mission_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  double dt_;

  SolverTypes::Type solver_type_;
  // crocoddyl::SolverBoxFDDP solver_BoxFDDP_;
  boost::shared_ptr<ProblemMission> problem_;

};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_