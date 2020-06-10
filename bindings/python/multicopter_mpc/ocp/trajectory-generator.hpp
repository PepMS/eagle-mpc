#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_

#include "multicopter_mpc/ocp/trajectory-generator.hpp"

#include <boost/python.hpp>

namespace multicopter_mpc {
namespace python {
namespace bp = boost::python;

void exposeTrajectoryGenerator() {
  bp::register_ptr_to_python<boost::shared_ptr<TrajectoryGenerator>>();

  bp::class_<TrajectoryGenerator>(
      "TrajectoryGenerator",
      bp::init<const boost::shared_ptr<pinocchio::Model>, const boost::shared_ptr<MultiCopterBaseParams>&,
               const double&, const boost::shared_ptr<Mission>&>(bp::args("self", "mc_params", "dt", "mission"),
                                                                 "Initialize trajectory generation OCP"));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_TRAJECTORY_GENERATOR_HPP_