#ifndef MULTICOPTER_MPC_MPC_MAIN_HPP_
#define MULTICOPTER_MPC_MPC_MAIN_HPP_

#define MOTOR_TH_NORM_MIN -1.0
#define MOTOR_TH_NORM_MAX 1.0

#include <string>

#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/solver-base.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"

#include "yaml_parser/parser_yaml.h"
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
  MpcMain();
  ~MpcMain();

  // Problem management related
  void removeProblem();
  void createProblem();
  const boost::shared_ptr<const crocoddyl::SolverAbstract> getSolver() const;

  // MPC Run related
  void solve(const size_t& n_iter);
  void setInitialState(const Eigen::Ref<const Eigen::VectorXd>& initial_state);
  const Eigen::VectorXd& getActuatorControls() const;
  const Eigen::VectorXd& getActuatorControlsNormalized() const;
  const Eigen::VectorXd& getState(const size_t& n_node) const;

  boost::shared_ptr<Mission> mission_;

 private:
  void initBoxFDDP();
  void initSquashBoxFDDP();
  void init();
  void computeNormalizedControls();

  MultiCopterTypes::Type mc_type_;
  MissionTypes::Type mission_type_;
  std::string model_frame_name_;

  // DDP related
  boost::shared_ptr<pinocchio::Model> model_;
  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<MultiCopterBaseParams> params_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  boost::shared_ptr<crocoddyl::SolverAbstract> solver_;
  boost::shared_ptr<crocoddyl::ShootingProblem> problem_opt_;

  double dt_;
  Eigen::VectorXd controls_normalized_;
  Eigen::VectorXd controls_;

  SolverTypes::Type solver_type_;
  boost::movelib::unique_ptr<ProblemMission> problem_;
};

}  // namespace multicopter_mpc
#endif  // MULTICOPTER_MPC_MPC_MAIN_HPP_