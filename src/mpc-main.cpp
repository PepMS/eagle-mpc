#include "multicopter_mpc/mpc-main.hpp"

namespace multicopter_mpc {
MpcMain::MpcMain(MultiCopterTypes mc_type, MissionTypes mission_type, SolverTypes solver_type)
    : mc_type_(mc_type), mission_type_(mission_type), solver_type_(solver_type) {
  std::string model_path;
  std::string model_frame_name;
  yaml_parser::ParserYAML yaml_file_mc;

  switch (mc_type_.Type) {
    case MultiCopterTypes::Iris:
      model_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf" yaml_file_mc =
          yaml_parser::ParserYAML("/usr/local/share/multicopter_mpc/multirotor/iris.yaml", "", true);
      break;
    case MultiCopterTypes::Hector:
      model_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf" yaml_file_mc =
          yaml_parser::ParserYAML("/usr/local/share/multicopter_mpc/multirotor/hector.yaml", "", true);
      break;
    default:
      break;
  }
}
}  // namespace multicopter_mpc