#ifndef BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_

#include "multicopter_mpc/trajectory.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeTrajectory() {
  bp::register_ptr_to_python<boost::shared_ptr<Trajectory>>();

  bp::class_<Trajectory, boost::shared_ptr<Trajectory>>("Trajectory", bp::no_init)
      // .add_property("create", bp::make_function(&Trajectory::create, bp::return_value_policy<bp::return_internal_reference>()));
      .def("__init__", bp::make_constructor(&Trajectory::create))
      .def("createProblem", &Trajectory::createProblem, bp::args("self", "dt", "squash", "x0", "integration_method"))
      .def("autoSetup", &Trajectory::autoSetup, bp::args("self", "server"));
}
}  // namespace python
}  // namespace multicopter_mpc
#endif