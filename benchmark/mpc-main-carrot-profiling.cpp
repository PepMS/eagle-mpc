#include <string>

#include "utils.hpp"

int main(int argc, char* argv[]) {
  std::string mission_type = "passthrough.yaml";
  std::string mpc_main_yaml_path = MULTICOPTER_MPC_OCP_DIR "/mpc-main.yaml";

  multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, mission_type, mpc_main_yaml_path);
  Simulator simulator(mpc_main.getMpcController()->getTimeStep());

  Eigen::VectorXd state = mpc_main.getMpcController()->getStateMultibody()->zero();
  Eigen::VectorXd control = Eigen::VectorXd::Zero(mpc_main.getMpcController()->getMcParams()->n_rotors_);

  for (std::size_t i = 0; i < mpc_main.getMpcController()->getTrajectoryGenerator()->getKnots(); ++i) {
    // applyRandomNoise(state);
    mpc_main.setCurrentState(state);
    mpc_main.runMpcStep();
    control = mpc_main.getMotorsThrust();
    state = simulator.simulateStep(state, control);
    if (i % 20 == 0) {
      std::cout << "Step number: " << i << std::endl;
    }
  }
}