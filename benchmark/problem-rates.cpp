#include <iostream>
#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"

#include "crocoddyl/core/utils/timer.hpp"

#include "example-robot-data/path.hpp"

#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/path.h"

void applyRandomNoise(Eigen::VectorXd& vector) {
  Eigen::VectorXd r_vector = Eigen::VectorXd::Random(vector.size());

  vector += r_vector * 0.05;

  Eigen::Quaterniond quat;
  quat.vec() = vector.segment(3, 3);
  quat.w() = vector(6);
  quat.normalize();

  vector.segment(3, 3) = quat.vec();
  vector(6) = quat.w();
}

void printNodeCombination(const int& nodes_number) { std::cout << "NUMBER OF NODES: " << nodes_number << std::endl; }

void printTrial(const int& trial_number, const int& iter_number, const double& elapsed_time, const double& stop,
                const double& cost) {
  std::string separator = ". ";

  std::cout << "Trial # " << trial_number << ": ";

  std::cout << "Iterations: " << iter_number << separator;
  std::cout << "Elapsed time: " << elapsed_time << separator;
  std::cout << "Freq. per iteration: " << iter_number / elapsed_time * 1000 << separator;
  std::cout << "Stop value: " << stop << separator;
  std::cout << "Final cost: " << cost << std::endl;
}

void printOverview(const Eigen::ArrayXd& iterations, const Eigen::ArrayXd& durations, const int& trial_num) {
  std::string separator = ". ";

  std::cout << "Average: ";

  std::cout << "Iterations: " << iterations.sum() / trial_num << separator;
  std::cout << "Elapsed time: " << durations.sum() / trial_num << separator;

  Eigen::ArrayXd time_weighted;
  time_weighted = durations / iterations;
  std::cout << "Freq. per iteration: " << trial_num / time_weighted.sum() * 1000.0 << std::endl;
}

int main(void) {
  // int nodes_max = 200;
  // int nodes_min = 30;
  // int nodes_inc = 10;

  // int solve_trials = 10;

  // for (int n = nodes_min; n < nodes_max; n += nodes_inc) {
  //   std::string model_description_path;
  //   std::string model_yaml_path;
  //   std::string mission_yaml_path = MULTICOPTER_MPC_MISSION_DIR "/hover.yaml";

  //   model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
  //   model_yaml_path = MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml";

  //   yaml_parser::ParserYAML yaml_mc(model_yaml_path, "", true);
  //   yaml_parser::ParamsServer server_params(yaml_mc.getParams());

  //   yaml_parser::ParserYAML yaml_mission(mission_yaml_path, "", true);
  //   yaml_parser::ParamsServer server_mission(yaml_mission.getParams());

  //   pinocchio::Model model;
  //   pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
  //   boost::shared_ptr<pinocchio::Model> model_ = boost::make_shared<pinocchio::Model>(model);
  //   boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_ =
  //       boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  //   mc_params_->fill(server_params);
  //   boost::shared_ptr<multicopter_mpc::Mission> mission_ = boost::make_shared<multicopter_mpc::Mission>(model_->nq + model_->nv);
  //   mission_->fillWaypoints(server_mission);
  //   mission_->fillInitialState(server_mission);

  //   double dt_ = 4e-3;
  //   multicopter_mpc::TrajectoryGenerator trajectory_generator(model_, mc_params_,dt_,mission_);
   
  //   // mission_->getWaypoints()[0].knots = n;

  //   trajectory_generator.createProblem(multicopter_mpc::SolverTypes::BoxFDDP);

  //   printNodeCombination(n);
  //   Eigen::ArrayXd durations(solve_trials);
  //   Eigen::ArrayXd iterations(solve_trials);
  //   trajectory_generator.solve();
  //   for (int i = 0; i < solve_trials; i++) {
  //     Eigen::VectorXd initial_state = mission_->getInitialState();
  //     applyRandomNoise(initial_state);
  //     crocoddyl::Timer timer;
  //     trajectory_generator.setInitialState(initial_state);
  //     trajectory_generator.solve();
  //     durations[i] = timer.get_duration();
  //     iterations[i] = trajectory_generator.getSolver()->get_iter();
  //     printTrial(i, trajectory_generator.getSolver()->get_iter(), durations[i], trajectory_generator.getSolver()->get_stop(),
  //                trajectory_generator.getSolver()->get_cost());
  //   }
  //   printOverview(iterations, durations, solve_trials);
  // }
}