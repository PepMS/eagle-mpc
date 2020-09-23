#include <iostream>

#include <Eigen/Dense>

#include "crocoddyl/core/utils/timer.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/mpc-main.hpp"

#define SMOOTH(s) for (std::size_t _smooth = 0; _smooth < s; ++_smooth)

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / ((double)vec.size() - 1))
#define AVG(vec) (vec.mean())

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

int main(int argc, char* argv[]) {
  unsigned int T = 1500;
  crocoddyl::Timer timer;

  std::string mission_type = "passthrough.yaml";
  std::string mpc_main_yaml_path = MULTICOPTER_MPC_OCP_DIR "/mpc-main.yaml";

  multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, mission_type, mpc_main_yaml_path);

  Eigen::VectorXd state = mpc_main.getMpcController()->getStateMultibody()->zero();

  std::cout << "Function call: \t\t\t\t"
            << "AVG(in us)\t"
            << "STDDEV(in us)\t"
            << "MAX(in us)\t"
            << "MIN(in us)" << std::endl;

  Eigen::ArrayXd duration(T);
  SMOOTH(T) {
    applyRandomNoise(state);
    timer.reset();
    mpc_main.setCurrentState(state);
    duration[_smooth] = timer.get_us_duration();
  }
  std::cout << "setCurrentState :\t\t\t" << AVG(duration) << " us\t" << STDDEV(duration) << " us\t"
            << duration.maxCoeff() << " us\t" << duration.minCoeff() << " us" << std::endl;

  duration.setZero();
  SMOOTH(T) {
    applyRandomNoise(state);
    mpc_main.setCurrentState(state);
    timer.reset();
    mpc_main.runMpcStep();
    duration[_smooth] = timer.get_us_duration();
    state = mpc_main.getMpcController()->getSolver()->get_xs()[1];
  }
  std::cout << "runMpcStep :\t\t\t\t" << AVG(duration) << " us\t" << STDDEV(duration) << " us\t" << duration.maxCoeff()
            << " us\t" << duration.minCoeff() << " us" << std::endl;

    std::cout << "This is the final state: \n" << state << std::endl;
}