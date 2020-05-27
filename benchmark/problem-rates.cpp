#include <iostream>
#include <Eigen/Dense>

#include "crocoddyl/core/utils/timer.hpp"

#include "multicopter_mpc/mpc-main.hpp"

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
  int nodes_max = 200;
  int nodes_min = 30;
  int nodes_inc = 10;

  int solve_trials = 10;

  for (int n = nodes_min; n < nodes_max; n += nodes_inc) {
    multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, multicopter_mpc::MissionTypes::Hover,
                                      multicopter_mpc::SolverTypes::BoxFDDP);
    
    mpc_main.mission_->waypoints_[0].knots = n;
    
    mpc_main.createProblem();

    printNodeCombination(n);
    Eigen::ArrayXd durations(solve_trials);
    Eigen::ArrayXd iterations(solve_trials);
    mpc_main.solve(20);
    for (int i = 0; i < solve_trials; i++) {
      Eigen::VectorXd initial_state = mpc_main.mission_->x0_;
      applyRandomNoise(initial_state);
      crocoddyl::Timer timer;
      mpc_main.setInitialState(initial_state);
      mpc_main.solve(20);
      durations[i] = timer.get_duration();
      iterations[i] = mpc_main.getSolver()->get_iter();
      printTrial(i, mpc_main.getSolver()->get_iter(), durations[i], mpc_main.getSolver()->get_stop(),
                 mpc_main.getSolver()->get_cost());
    }
    printOverview(iterations, durations, solve_trials);
    // if (n == 100) {
    //   std::cout << "This is the initial state: " << std::endl;
    //   std::cout << mpc_main.getSolver()->get_xs()[0] << std::endl;
    //   std::cout << "This is the final state: " << std::endl;
    //   std::cout << mpc_main.getSolver()->get_xs().back() << std::endl;
    //   std::cout << "Control sequence: " << std::endl;
    //   for (int j = 0; j < mpc_main.getSolver()->get_us().size(); ++j) {
    //     std::cout << "Control " << j << std::endl;
    //     std::cout << mpc_main.getSolver()->get_us()[j] << std::endl;
    //   }
    // }
  }
}