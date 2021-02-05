#ifndef BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_

#include "multicopter_mpc/trajectory.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"

#include "crocoddyl/multibody/states/multibody.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeTrajectory() {
  StdVectorPythonVisitor<boost::shared_ptr<Stage>, std::allocator<boost::shared_ptr<Stage>>, true>::expose(
      "StdVec_Stages");

  bp::register_ptr_to_python<boost::shared_ptr<crocoddyl::StateMultibody>>();

  bp::class_<Trajectory, boost::shared_ptr<Trajectory>>("Trajectory", bp::no_init)
      .def("__init__", bp::make_constructor(&Trajectory::create))
      .add_property("stages",
                    bp::make_function(&Trajectory::get_stages, bp::return_value_policy<bp::return_by_value>()))
      .add_property("robot_model",
                    bp::make_function(&Trajectory::get_robot_model, bp::return_value_policy<bp::return_by_value>()))
      .add_property("platform_params", bp::make_function(&Trajectory::get_platform_params,
                                                         bp::return_value_policy<bp::return_by_value>()))
      .add_property("state",
                    bp::make_function(&Trajectory::get_robot_state, bp::return_value_policy<bp::return_by_value>()))
      .add_property("actuation",
                    bp::make_function(&Trajectory::get_actuation, bp::return_value_policy<bp::return_by_value>()))
      .add_property("actuation_squash", bp::make_function(&Trajectory::get_actuation_squash,
                                                          bp::return_value_policy<bp::return_by_value>()))
      .add_property("squash",
                    bp::make_function(&Trajectory::get_squash, bp::return_value_policy<bp::return_by_value>()))
      .def("createProblem", &Trajectory::createProblem, bp::args("self", "dt", "squash", "x0", "integration_method"))
      .def("autoSetup", &Trajectory::autoSetup, bp::args("self", "server"));
}
}  // namespace python
}  // namespace multicopter_mpc
#endif