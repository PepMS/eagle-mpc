#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/solvers/ddp.hpp"

#include "yaml_parser/parser_yaml.h"

#include "multicopter_mpc/multicopter-base-params.hpp"
#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/path.h"

int main(void) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",
                              pinocchio::JointModelFreeFlyer(), model);

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params =
      boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
  mc_params->fill(MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml");

  boost::shared_ptr<multicopter_mpc::Mission> mission =
      boost::make_shared<multicopter_mpc::Mission>(model.nq + model.nv);
  mission->fillWaypoints(MULTICOPTER_MPC_MISSION_DIR "/passthrough.yaml");

  boost::shared_ptr<pinocchio::Model> mc_model = boost::make_shared<pinocchio::Model>(model);

  multicopter_mpc::TrajectoryGenerator trajectory(mc_model, mc_params, mission);
  trajectory.loadParameters(MULTICOPTER_MPC_OCP_DIR "/trajectory-generator.yaml");
  trajectory.createProblem(multicopter_mpc::SolverTypes::BoxFDDP, multicopter_mpc::IntegratorTypes::RK4, 1e-2);
  printf("This is the time step %f \n", trajectory.getTimeStep());
  printf("This is the knot number %d \n", trajectory.getKnots());

  trajectory.setSolverCallbacks(true);
  trajectory.solve();
}