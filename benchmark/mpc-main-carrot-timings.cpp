#include <iostream>

#include <Eigen/Dense>

#include "crocoddyl/core/utils/timer.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/mpc-main.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
  crocoddyl::Timer timer;

  std::string mission_type = "passthrough.yaml";
  std::string mpc_main_yaml_path = MULTICOPTER_MPC_OCP_DIR "/mpc-main.yaml";

  multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, mission_type, mpc_main_yaml_path);
  Simulator simulator(mpc_main.getMpcController()->getTimeStep());

  Eigen::VectorXd state = mpc_main.getMpcController()->getStateMultibody()->zero();
  Eigen::VectorXd control = Eigen::VectorXd::Zero(mpc_main.getMpcController()->getMcParams()->n_rotors_);

  std::size_t T = mpc_main.getMpcController()->getTrajectoryGenerator()->getKnots();
  Eigen::ArrayXd duration(T);
  Eigen::ArrayXd iterations(T);
  Eigen::ArrayXd duration_iteration(T);
  duration.setZero();
  iterations.setZero();
  for (std::size_t i = 0; i < T; ++i) {
    timer.reset();
    mpc_main.setCurrentState(state);
    mpc_main.runMpcStep();
    control = mpc_main.getMotorsThrust();
    duration[i] = timer.get_us_duration();
    iterations[i] = mpc_main.getMpcController()->getSolver()->get_iter();
    duration_iteration[i] = duration[i] / iterations[i];
    state = simulator.simulateStep(state, control);
    if (i % 20 == 0) {
      std::cout << "Step number: " << i << std::endl;
    }
  }
  std::cout << "Function call: \t\t\t\t"
            << "AVG(in us)\t"
            << "STDDEV(in us)\t"
            << "MAX(in us)\t"
            << "MIN(in us)" << std::endl;

  std::cout << "runMpcStep: \t\t\t\t" << AVG(duration) << " us\t" << STDDEV(duration) << " us\t" << duration.maxCoeff()
            << " us\t" << duration.minCoeff() << " us" << std::endl;

  std::cout << "Avg.time per iteration: \t\t" << AVG(duration_iteration) << " us\t" << STDDEV(duration_iteration) << " us\t"
            << duration_iteration.maxCoeff() << " us\t" << duration_iteration.minCoeff() << " us" << std::endl;

  std::cout << "This is the final state: \n" << state << std::endl;
}