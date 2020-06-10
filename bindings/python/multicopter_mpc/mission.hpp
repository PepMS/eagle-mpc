#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_

#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/waypoint.hpp"

#include "python/multicopter_mpc/vector-converter.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeMission() {
  bp::to_python_converter<std::vector<WayPoint, std::allocator<WayPoint>>, vector_to_list<WayPoint, true>>();
  list_to_vector().from_python<std::vector<WayPoint, std::allocator<WayPoint>>>();

  bp::register_ptr_to_python<boost::shared_ptr<Mission>>();

  bp::class_<Mission>("Mission", bp::init<int>(bp::args("self", "nx"), "Initialize mission params"))
      .def("fillWaypoints", &Mission::fillWaypoints, bp::args("self", "server"))
      .def("fillInitialState", &Mission::fillInitialState, bp::args("self", "server"))
      .add_property("x0", bp::make_getter(&Mission::x0_, bp::return_internal_reference<>()),
                    bp::make_setter(&Mission::x0_), "Initial state")
      .add_property("waypoints", bp::make_getter(&Mission::waypoints_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&Mission::waypoints_), "List of waypoints")
      .add_property("total_knots", bp::make_function(&Mission::getTotalKnots, bp::return_value_policy<bp::return_by_value>()));
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_