#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_

#include "multicopter_mpc/mission.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeMission() {
  bp::class_<Mission>("Mission", bp::init<int>(bp::args("nx"), "Initialize mission params"))
      .def("fillWaypoints", &Mission::fillWaypoints, bp::args("server"))
      .def("fillInitialState", &Mission::fillInitialState, bp::args("server"))
      .add_property("x0", bp::make_getter(&Mission::x0_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&Mission::x0_), "Initial state")
      .add_property("waypoints", bp::make_getter(&Mission::waypoints_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&Mission::waypoints_), "Waypoints list");
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_