#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_

#include "multicopter_mpc/ocp/trajectory-generator.hpp"

#include <boost/python.hpp>

namespace multicopter_mpc {
namespace python {
namespace bp = boost::python;

void exposeTrajectoryGenerator() {
  bp::register_ptr_to_python<boost::shared_ptr<TrajectoryGenerator>>();
  void (TrajectoryGenerator::*solve_void)(void) = &TrajectoryGenerator::solve;
  void (TrajectoryGenerator::*solve_init)(const std::vector<Eigen::VectorXd>&,
                                          const std::vector<Eigen::VectorXd>&) =
      &TrajectoryGenerator::solve;
  bp::class_<TrajectoryGenerator, bp::bases<OcpAbstract>>(
      "TrajectoryGenerator",
      bp::init<const boost::shared_ptr<pinocchio::Model>, const boost::shared_ptr<MultiCopterBaseParams>&,
               const double&, const boost::shared_ptr<Mission>&>(
          bp::args("self", "model", "mc_params", "dt", "mission"), "Initialize trajectory generation OCP"))
      .def("createProblem", &TrajectoryGenerator::createProblem, bp::args("self", "solver_type"))
      .def("createCostStateRegularization", &TrajectoryGenerator::createCostStateRegularization, bp::args("self"))
      .def("createCostControlRegularization", &TrajectoryGenerator::createCostControlRegularization, bp::args("self"))
      .def("loadParameters",&TrajectoryGenerator::loadParameters, bp::args("self","yaml_path"))
      .def("solve", solve_void, bp::args("self"))
      .def("solve", solve_init, bp::args("self", "state_trajectory", "control_trajectory"))
      .add_property("mission", bp::make_function(&TrajectoryGenerator::getMission,
                                                 bp::return_value_policy<bp::return_by_value>()))
      .def("getStateTrajectory", &TrajectoryGenerator::getStateTrajectory, bp::args("self", "idx_init", "idx_end"))
      .def("getControlTrajectory", &TrajectoryGenerator::getControlTrajectory, bp::args("self", "idx_init", "idx_end"))
      .def("getState", &TrajectoryGenerator::getState, bp::return_internal_reference<>(), bp::args("self", "cursor"));
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_