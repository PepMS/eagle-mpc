///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_BASE_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_BASE_HPP_

#include "eagle_mpc/mpc-base.hpp"
#include "python/eagle_mpc/utils/vector-converter.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

class MpcAbstract_wrap : public MpcAbstract, public bp::wrapper<MpcAbstract>
{
    public:
    MpcAbstract_wrap(const std::string& yaml_path) : MpcAbstract(yaml_path), bp::wrapper<MpcAbstract>() {}

    void createProblem() { this->get_override("createProblem")(); }
    void updateProblem(const std::size_t& current_time) { this->get_override("updateProblem")(); }
};

void exposeMpcAbstract()
{
    bp::enum_<MpcTypes>("MpcTypes")
        .value("Carrot", MpcTypes::Carrot)
        .value("Rail", MpcTypes::Rail)
        .value("Weighted", MpcTypes::Weighted);

    bp::class_<MpcAbstract_wrap, boost::noncopyable>(
        "MpcAbstract", "Abstract class to generate an MPC Controller to run on multicopter or aerial manipulators",
        bp::init<const std::string&>(bp::args("self", "yaml_path"), "Initialize the MPC Controller abstract class"))
        .def("createProblem", bp::pure_virtual(&MpcAbstract::createProblem), bp::args("self"))
        .def("updateProblem", bp::pure_virtual(&MpcAbstract::updateProblem), bp::args("self"))
        .add_property("robot_model", bp::make_function(&MpcAbstract_wrap::get_robot_model,
                                                       bp::return_value_policy<bp::return_by_value>()))
        .add_property("robot_model_path", bp::make_function(&MpcAbstract_wrap::get_robot_model_path,
                                                            bp::return_value_policy<bp::return_by_value>()))
        .add_property("platform_params", bp::make_function(&MpcAbstract_wrap::get_platform_params,
                                                           bp::return_value_policy<bp::return_by_value>()))
        .add_property("state", bp::make_function(&MpcAbstract_wrap::get_robot_state,
                                                 bp::return_value_policy<bp::return_by_value>()))
        .add_property("actuation", bp::make_function(&MpcAbstract_wrap::get_actuation,
                                                     bp::return_value_policy<bp::return_by_value>()))
        .add_property("actuation_squash", bp::make_function(&MpcAbstract_wrap::get_actuation_squash,
                                                            bp::return_value_policy<bp::return_by_value>()))
        .add_property("squash",
                      bp::make_function(&MpcAbstract_wrap::get_squash, bp::return_value_policy<bp::return_by_value>()))
        .add_property(
            "dif_models",
            bp::make_function(&MpcAbstract_wrap::get_dif_models, bp::return_value_policy<bp::return_by_value>()),
            "return the problem differential models")
        .add_property(
            "int_models",
            bp::make_function(&MpcAbstract_wrap::get_int_models, bp::return_value_policy<bp::return_by_value>()),
            "return the problem integrated models")
        .add_property("problem", bp::make_function(&MpcAbstract_wrap::get_problem,
                                                   bp::return_value_policy<bp::return_by_value>()))
        .add_property("solver",
                      bp::make_function(&MpcAbstract_wrap::get_solver, bp::return_value_policy<bp::return_by_value>()))
        .add_property("knots",
                      bp::make_function(&MpcAbstract_wrap::get_knots, bp::return_value_policy<bp::return_by_value>()))
        .add_property("dt",
                      bp::make_function(&MpcAbstract_wrap::get_dt, bp::return_value_policy<bp::return_by_value>()))
        .add_property("iters",
                      bp::make_function(&MpcAbstract_wrap::get_iters, bp::return_value_policy<bp::return_by_value>()));
}

}  // namespace python
}  // namespace eagle_mpc
#endif