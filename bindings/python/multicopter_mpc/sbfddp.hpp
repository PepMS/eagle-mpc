#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_SBFDDP_BASE_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_SBFDDP_BASE_HPP_

#include <boost/python.hpp>
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"

#include "multicopter_mpc/sbfddp.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeSolverSbFDDP() {
  bp::register_ptr_to_python<boost::shared_ptr<SolverSbFDDP> >();

  bp::class_<SolverSbFDDP, bp::bases<crocoddyl::SolverFDDP> >(
      "SolverSbFDDP",
      "Box-constrained FDDP solver.\n\n"
      ":param shootingProblem: shooting problem (list of action models along trajectory.)",
      bp::init<boost::shared_ptr<crocoddyl::ShootingProblem>, boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> >(
          bp::args("self", "problem"),
          "Initialize the vector dimension.\n\n"
          ":param problem: shooting problem."));

}  // namespace python
}  // namespace python

}  // namespace multicopter_mpc

#endif