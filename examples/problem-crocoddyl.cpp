#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/mission.hpp"

int main(void) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  yaml_parser::ParserYAML yaml_file("../../config/multirotor/hector.yaml", "", true);
  yaml_parser::ParamsServer server_params(yaml_file.getParams());

  multicopter_mpc::MultiCopterBaseParams mc_params;
  mc_params.fill(server_params);

  yaml_parser::ParserYAML yaml_mission("../../config/mission/simple.yaml", "", true);
  yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  multicopter_mpc::Mission mission(model.nq + model.nv);
  mission.fillWaypoints(server_mission);
  mission.fillInitialState(server_mission);

  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model));

  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> act_model =
      boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state, mc_params.n_rotors_, mc_params.tau_f_);

  boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
      boost::make_shared<crocoddyl::CostModelSum>(state, act_model->get_nu());

  crocoddyl::FramePlacement M_ref =
      crocoddyl::FramePlacement(model.getFrameId("base_link"), mission.waypoints_[0].pose);
  Eigen::VectorXd state_weights = Eigen::VectorXd(model.nv * 2);
  state_weights.head(3).fill(0.1);
  state_weights.tail(model.nv * 2 - 3).fill(1000);

  boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
      boost::make_shared<crocoddyl::CostModelFramePlacement>(state, M_ref, act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost = boost::make_shared<crocoddyl::CostModelState>(
      state, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(state_weights), state->zero(),
      act_model->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> control_reg_cost =
      boost::make_shared<crocoddyl::CostModelControl>(state, act_model->get_nu());

  running_cost_model->addCost("x_reg", state_reg_cost, 1e-6);
  running_cost_model->addCost("u_reg", control_reg_cost, 1e-6);
  running_cost_model->addCost("track_pose", goal_tracking_cost, 1e-2);
  terminal_cost_model->addCost("goal_pose", goal_tracking_cost, 100);

  double dt = 3e-2;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
          boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model, running_cost_model),
          dt);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
          boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, act_model, terminal_cost_model),
          dt);

  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models(mission.waypoints_[0].knots,
                                                                                running_model);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(mission.x0_, running_models, terminal_model);

  crocoddyl::SolverFDDP fddp(problem);
  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;
  cbs.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  fddp.setCallbacks(cbs);

  fddp.solve();
  fddp.set_stoppingCriteria(fddp.StopCriteriaCostReduction);
  fddp.solve();

}