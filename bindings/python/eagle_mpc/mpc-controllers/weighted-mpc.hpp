///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_

#include "eagle_mpc/mpc-controllers/weighted-mpc.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

void exposeWeightedMpc()
{
    bp::register_ptr_to_python<boost::shared_ptr<WeightedMpc>>();
    bp::class_<WeightedMpc, bp::bases<MpcAbstract>>(
        "WeightedMpc", bp::init<const boost::shared_ptr<Trajectory>&, const std::size_t, const std::string&>())
        .def("createProblem", &WeightedMpc::createProblem, bp::args("self"))
        .def("updateProblem", &WeightedMpc::updateProblem, bp::args("self", "currentTime"))
        .add_property("trajectory",
                      bp::make_function(&WeightedMpc::get_trajectory, bp::return_value_policy<bp::return_by_value>()))
        .add_property(
            "t_stages",
            bp::make_function(&WeightedMpc::get_t_stages, bp::return_value_policy<bp::copy_const_reference>()),
            "time trajectory");
}

}  // namespace python
}  // namespace eagle_mpc
#endif