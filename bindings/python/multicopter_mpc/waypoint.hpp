#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_WAYPOINT_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_WAYPOINT_HPP_

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "multicopter_mpc/waypoint.hpp"
#include "python/multicopter_mpc/utils/optional-converter.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeWayPoint() {
  bp::register_exception_translator<AttributeError>(&translate);
  bp::register_exception_translator<TypeError>(&translate);
  bp::to_python_converter<boost::optional<pinocchio::Motion>, to_python_optional<pinocchio::Motion> >();
  from_python_optional<pinocchio::Motion>();

  bp::register_ptr_to_python<boost::shared_ptr<WayPoint> >();

  bp::class_<WayPoint>("WayPoint",
                       bp::init<std::size_t, Eigen::Vector3d, Eigen::Quaterniond>(
                           bp::args("self", "knots", "position", "orientation"), "Initialize params server"))
      .def(bp::init<std::size_t, Eigen::Vector3d, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d>(
          bp::args("self", "knots", "position", "orientation", "velocity", "rates")))
      .add_property("knots", bp::make_getter(&WayPoint::knots, bp::return_value_policy<bp::return_by_value>()))
      .add_property("pose", bp::make_getter(&WayPoint::pose, bp::return_internal_reference<>()))
      .add_property("velocity", bp::make_getter(&WayPoint::vel, bp::return_value_policy<bp::return_by_value>()))
      .add_property("time", bp::make_getter(&WayPoint::time, bp::return_value_policy<bp::return_by_value>()));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_WAYPOINT_HPP_