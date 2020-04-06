#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_ALGORITHMS_WAYPOINT_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_ALGORITHMS_WAYPOINT_HPP_

#include "multicopter_mpc/waypoint.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeWayPoint() {
  bp::class_<WayPoint>("WayPoint", bp::init<int, Eigen::Vector3d&, Eigen::Quaterniond&>(
                                       bp::args("knots, position, orientation"), "Initialize params server"))
      .add_property("M", bp::make_getter(&WayPoint::pose, bp::return_value_policy<bp::return_by_value>()))
      .add_property("twist", bp::make_getter(&WayPoint::vel, bp::return_value_policy<bp::return_by_value>()))
      .add_property("knots", bp::make_getter(&WayPoint::knots, bp::return_value_policy<bp::return_by_value>()));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_OPTIUAVM_ALGORITHMS_WAYPOINT_HPP_