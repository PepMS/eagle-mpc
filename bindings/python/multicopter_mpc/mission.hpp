#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_

#include "multicopter_mpc/mission.hpp"
#include "multicopter_mpc/waypoint.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeMission() {
  bp::to_python_converter<std::vector<WayPoint, std::allocator<WayPoint>>, vector_to_list<WayPoint, true>>();
  list_to_vector().from_python<std::vector<WayPoint, std::allocator<WayPoint>>>();

  bp::register_ptr_to_python<boost::shared_ptr<Mission>>();

  void (Mission::*fillWP_yaml)(const std::string&) = &Mission::fillWaypoints;
  void (Mission::*fillWP_yaml_dt)(const std::string&, const double&) = &Mission::fillWaypoints;

  bp::class_<Mission>("Mission", bp::init<int>(bp::args("self", "nx"), "Initialize mission params"))
      .def("fillWaypoints", fillWP_yaml, bp::args("self", "yaml_path"))
      .def("fillWaypoints", fillWP_yaml_dt, bp::args("self", "yaml_path", "dt"))
      .def("interpolateTrajectory", &Mission::interpolateTrajectory, bp::args("self", "inter_type"))
      .add_property("x0", bp::make_function(&Mission::getInitialState, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_function(&Mission::setInitialState), "Initial state")
      .add_property("waypoints",
                    bp::make_function(&Mission::getWaypoints, bp::return_value_policy<bp::return_by_value>()))
      .add_property("total_knots",
                    bp::make_function(&Mission::getTotalKnots, bp::return_value_policy<bp::return_by_value>()))
      .add_property("dt", bp::make_function(&Mission::getTimeStep, bp::return_value_policy<bp::return_by_value>()));
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_MISSION_HPP_