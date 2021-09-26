///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_SBFDDP_BASE_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_SBFDDP_BASE_HPP_

#include <boost/python.hpp>
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"

#include "eagle_mpc/sbfddp.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SolverSbFDDP_solves, SolverSbFDDP::solve, 0, 5)

void exposeSolverSbFDDP()
{
    bp::register_ptr_to_python<boost::shared_ptr<SolverSbFDDP>>();

    const std::vector<Eigen::VectorXd>& (SolverSbFDDP::*get_controls)() const = &SolverSbFDDP::get_us;
    const std::vector<Eigen::VectorXd>& (SolverSbFDDP::*get_controls_squash)() const =
        &SolverSbFDDP::getSquashControls;
    const std::vector<Eigen::VectorXd>& (SolverSbFDDP::*get_states)() const = &SolverSbFDDP::get_xs;
    std::size_t (SolverSbFDDP::*get_iter)() const                           = &SolverSbFDDP::get_iter;
    const boost::shared_ptr<crocoddyl::ShootingProblem>& (SolverSbFDDP::*getProblem)() const =
        &SolverSbFDDP::get_problem;

    bp::class_<SolverSbFDDP, bp::bases<crocoddyl::SolverFDDP>>(
        "SolverSbFDDP",
        "Box-constrained FDDP solver.\n\n"
        ":param shootingProblem: shooting problem (list of action models along trajectory.)",
        bp::init<boost::shared_ptr<crocoddyl::ShootingProblem>, boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>>(
            bp::args("self", "problem", "squashing"),
            "Initialize the vector dimension.\n\n"
            ":param problem: shooting problem."))
        .def(
            "solve", &SolverSbFDDP::solve,
            SolverSbFDDP_solves(
                bp::args("self", "init_xs", "init_us", "maxiter", "isFeasible", "regInit"),
                "Compute the optimal trajectory xopt, uopt as lists of T+1 and T terms.\n\n"
                "From an initial guess init_xs,init_us (feasible or not), iterate\n"
                "over computeDirection and tryStep until stoppingCriteria is below\n"
                "threshold. It also describes the globalization strategy used\n"
                "during the numerical optimization.\n"
                ":param init_xs: initial guess for state trajectory with T+1 elements (default [])\n"
                ":param init_us: initial guess for control trajectory with T elements (default []).\n"
                ":param maxiter: maximum allowed number of iterations (default 100).\n"
                ":param isFeasible: true if the init_xs are obtained from integrating the init_us (rollout) (default "
                "False).\n"
                ":param regInit: initial guess for the regularization value. Very low values are typical\n"
                "                used with very good guess points (init_xs, init_us) (default None).\n"
                ":returns the optimal trajectory xopt, uopt and a boolean that describes if convergence was reached."))
        .def("setCallbacks", &SolverSbFDDP::setCallbacks, bp::args("self"),
             "Set a list of callback functions using for diagnostic.\n\n"
             "Each iteration, the solver calls these set of functions in order to\n"
             "allowed user the diagnostic of the its performance.\n"
             ":param callbacks: set of callback functions.")
        .add_property("us", bp::make_function(get_controls, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("us_squash",
                      bp::make_function(get_controls_squash, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("xs", bp::make_function(get_states, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("problem", bp::make_function(getProblem, bp::return_value_policy<bp::copy_const_reference>()),
                      "shooting problem")
        .add_property("iter", bp::make_function(get_iter, bp::return_value_policy<bp::return_by_value>()),
                      "number of iterations runned in solve()")
        .add_property(
            "convergence_init",
            bp::make_function(&SolverSbFDDP::get_convergence_init, bp::return_value_policy<bp::return_by_value>()),
            bp::make_function(&SolverSbFDDP::set_convergence_init), "initial time of the stage in ms");
}  // namespace python
}  // namespace python

}  // namespace eagle_mpc

#endif