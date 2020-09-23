#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_MAIN_MPC_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_MAIN_MPC_HPP_

#include "multicopter_mpc/mpc-main.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeMpcMain() {
  bp::enum_<MultiCopterTypes::Type>("MultiCopterType")
      .value("Iris", MultiCopterTypes::Iris)
      .value("Hector", MultiCopterTypes::Hector);

  bp::register_ptr_to_python<boost::shared_ptr<MpcMain>>();

  bp::class_<MpcMain>("MpcMain", bp::init<const MultiCopterTypes::Type&, const std::string&, const std::string&>(
                                     bp::args("self", "multicopter_type", "mission_type", "yaml_path"), "MpcMain"))
      .def(bp::init<>(bp::args("self"), "Default initialization"))
      .def("setCurrentState", &MpcMain::setCurrentState, bp::args("self", "state"))
      .def("runMpcStep", &MpcMain::runMpcStep, bp::args("self"))
      .def("thrustToSpeed", &MpcMain::thrustToSpeed, bp::args("self", "motor_thrust", "motor_speed"))
      .add_property("mpc_controller",
                    bp::make_function(&MpcMain::getMpcController, bp::return_value_policy<bp::return_by_value>()))
      .add_property("motor_speed",
                    bp::make_function(&MpcMain::getMotorsSpeed, bp::return_value_policy<bp::return_by_value>()))
      .add_property("motor_thrust",
                    bp::make_function(&MpcMain::getMotorsThrust, bp::return_value_policy<bp::return_by_value>()))
      .add_property("ff_gains",
                    bp::make_function(&MpcMain::getFeedForwardGains, bp::return_value_policy<bp::return_by_value>()))
      .add_property("fb_gains",
                    bp::make_function(&MpcMain::getFeedBackGains, bp::return_value_policy<bp::return_by_value>()));
}
}  // namespace python
}  // namespace multicopter_mpc
#endif