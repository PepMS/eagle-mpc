#include <map>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"

#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/problem-mission.hpp"

int main(void) {
  pinocchio::Model model;
  // pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf",
  //                             pinocchio::JointModelFreeFlyer(), model);

  // yaml_parser::ParserYAML yaml_file("../../config/multirotor/hector.yaml", "", true);
  // yaml_parser::ParamsServer server_params(yaml_file.getParams());

  // multicopter_mpc::MultiCopterBaseParams mc_params;
  // mc_params.fill(server_params);

  // yaml_parser::ParserYAML yaml_mission("../../config/mission/simple.yaml", "", true);
  // yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  // multicopter_mpc::Mission mission(model.nq + model.nv);
  // mission.fillWaypoints(server_mission);
  // mission.fillInitialState(server_mission);

  // boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);
  // boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(mc_model);

  // Eigen::VectorXd s_lb = Eigen::VectorXd(mc_params.n_rotors_);
  // Eigen::VectorXd s_ub = Eigen::VectorXd(mc_params.n_rotors_);
  // s_lb.fill(mc_params.min_thrust_);
  // s_ub.fill(mc_params.max_thrust_);
  // boost::shared_ptr<crocoddyl::SquashingModelAbstract> squash =
  //     boost::make_shared<crocoddyl::SquashingModelSmoothSat>(s_lb, s_ub, mc_params.n_rotors_);
  // boost::shared_ptr<crocoddyl::ActuationModelAbstract> act_mc =
  //     boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(state, mc_params.n_rotors_, mc_params.tau_f_);
  // boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation =
  //     boost::make_shared<crocoddyl::ActuationSquashingModel>(act_mc, squash, act_mc->get_nu());

  // multicopter_mpc::ProblemMission multicopter_problem(
  //     boost::make_shared<multicopter_mpc::Mission>(mission),
  //     boost::make_shared<multicopter_mpc::MultiCopterBaseParams>(mc_params), mc_model, actuation,
  //     model.getFrameId("base_link"), 1e-2);

  // boost::shared_ptr<crocoddyl::ShootingProblem> problem = multicopter_problem.createProblem();

  // printf("Smooth parameter before: %f \n",
  //        boost::static_pointer_cast<crocoddyl::SquashingModelSmoothSat>(squash)->get_smooth());

  // boost::static_pointer_cast<crocoddyl::SquashingModelSmoothSat>(squash)->set_smooth(23.54);

  // for (int i = 0; i < problem->get_runningModels().size(); ++i) {
  //   boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> euler =
  //       boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem->get_runningModels()[i]);
  //   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> differential =
  //       boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler->get_differential());
  //   boost::shared_ptr<crocoddyl::ActuationSquashingModel> a_model =
  //       boost::static_pointer_cast<crocoddyl::ActuationSquashingModel>(differential->get_actuation());

  //   printf("Smooth parameter after: %f \n",
  //          boost::static_pointer_cast<crocoddyl::SquashingModelSmoothSat>(a_model->get_squashing())->get_smooth());
  // }

  // boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> euler =
  //     boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem->get_runningModels()[0]);
  // boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> differential =
  //     boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler->get_differential());

  // std::map<std::string, boost::shared_ptr<crocoddyl::CostItem>> cost_sum;
  // cost_sum = differential->get_costs()->get_costs();

  // printf("Weight before: %f \n", cost_sum["u_reg"]->weight);
  // cost_sum["u_reg"]->weight = 12.345;

  // for (int i = 0; i < problem->get_runningModels().size(); ++i) {
  //   boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> euler =
  //       boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(problem->get_runningModels()[i]);
  //   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> differential =
  //       boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(euler->get_differential());

  //   std::map<std::string, boost::shared_ptr<crocoddyl::CostItem>> cost_sum;
  //   cost_sum = differential->get_costs()->get_costs();
  //   printf("Weight before: %f \n", cost_sum["u_reg"]->weight);
  // }
}