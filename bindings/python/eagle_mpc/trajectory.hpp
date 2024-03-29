///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include "crocoddyl/multibody/states/multibody.hpp"

#include "eagle_mpc/trajectory.hpp"
#include "python/eagle_mpc/utils/vector-converter.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

void exposeTrajectory()
{
    StdVectorPythonVisitor<boost::shared_ptr<Stage>, std::allocator<boost::shared_ptr<Stage>>, true>::expose(
        "StdVec_Stages");

    bp::register_ptr_to_python<boost::shared_ptr<pinocchio::Model>>();

    boost::shared_ptr<crocoddyl::ShootingProblem> (Trajectory::*create_problem_no_args)() const =
        &Trajectory::createProblem;
    boost::shared_ptr<crocoddyl::ShootingProblem> (Trajectory::*create_problem_args)(
        const std::size_t&, const bool&, const std::string&) const = &Trajectory::createProblem;

    bp::class_<Trajectory, boost::shared_ptr<Trajectory>>("Trajectory", bp::no_init)
        .def("__init__", bp::make_constructor(&Trajectory::create))
        .add_property("stages",
                      bp::make_function(&Trajectory::get_stages, bp::return_value_policy<bp::return_by_value>()))
        .add_property("robot_model",
                      bp::make_function(&Trajectory::get_robot_model, bp::return_value_policy<bp::return_by_value>()))
        .add_property("robot_model_path", bp::make_function(&Trajectory::get_robot_model_path,
                                                            bp::return_value_policy<bp::return_by_value>()))
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
        .add_property("initial_state",
                      bp::make_function(&Trajectory::get_initial_state, bp::return_internal_reference<>()),
                      &Trajectory::set_initial_state, "initial state")
        .add_property("duration",
                      bp::make_function(&Trajectory::get_duration, bp::return_value_policy<bp::return_by_value>()),
                      "duration of the trajectory")
        .def("createProblem", create_problem_args, bp::args("self", "dt", "squash", "integration_method"))
        .def("createProblem", create_problem_no_args, bp::args("self"))
        .def("autoSetup", &Trajectory::autoSetup, bp::args("self", "yaml_path"))
        .def("removeStage", &Trajectory::removeStage, bp::args("self", "idx_stage"));
}
}  // namespace python
}  // namespace eagle_mpc
#endif