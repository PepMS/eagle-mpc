#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_

#include "multicopter_mpc/ocp/trajectory-generator.hpp"

#include <boost/python.hpp>

namespace multicopter_mpc {
namespace python {
namespace bp = boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TrajectoryGenerator_solves, OcpAbstract::solve, 0, 2)

void exposeTrajectoryGenerator() {
  bp::register_ptr_to_python<boost::shared_ptr<TrajectoryGenerator>>();
  bp::class_<TrajectoryGenerator, bp::bases<OcpAbstract>>(
      "TrajectoryGenerator",
      bp::init<const boost::shared_ptr<pinocchio::Model>, const boost::shared_ptr<MultiCopterBaseParams>&,
               const boost::shared_ptr<Mission>&>(bp::args("self", "model", "mc_params", "mission"),
                                                  "Initialize trajectory generation OCP"))
      .def("createCostStateRegularization", &TrajectoryGenerator::createCostStateRegularization, bp::args("self"))
      .def("createCostControlRegularization", &TrajectoryGenerator::createCostControlRegularization, bp::args("self"))
      .def("loadParameters", &TrajectoryGenerator::loadParameters, bp::args("self", "yaml_path"))
      .add_property("mission", bp::make_function(&TrajectoryGenerator::getMission,
                                                 bp::return_value_policy<bp::return_by_value>()))
      .def("getStateTrajectory", &TrajectoryGenerator::getStateTrajectory, bp::args("self", "idx_init", "idx_end"))
      .def("getControlTrajectory", &TrajectoryGenerator::getControlTrajectory, bp::args("self", "idx_init", "idx_end"))
      .def("getState", &TrajectoryGenerator::getState, bp::return_internal_reference<>(), bp::args("self", "cursor"))
      .def("solve", &TrajectoryGenerator::solve, TrajectoryGenerator_solves(bp::args("self", "init_xs", "init_us")));
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_