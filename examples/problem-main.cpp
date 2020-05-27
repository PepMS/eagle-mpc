#include <iostream>
#include <Eigen/Dense>

#include "multicopter_mpc/mpc-main.hpp"

int main(void)
{
  // multicopter_mpc::MpcMain mpc_main(multicopter_mpc::MultiCopterTypes::Iris, multicopter_mpc::MissionTypes::Hover, multicopter_mpc::SolverTypes::BoxFDDP);
  
  // mpc_main.solve();
  // std::cout << "Control output: \n" << mpc_main.getActuatorControls() << std::endl;

  // Eigen::VectorXd state(13);
  // state = Eigen::VectorXd::Zero(13);

  // state(6) = 1.0;
  // state(2) = 0.9;
  // mpc_main.setInitialState(state);
  // mpc_main.solve();
  // std::cout << "Control output: \n" << mpc_main.getActuatorControls() << std::endl;
}