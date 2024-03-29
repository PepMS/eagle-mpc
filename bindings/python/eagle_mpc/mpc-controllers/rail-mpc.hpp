
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_RAIL_MPC_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_RAIL_MPC_HPP_

#include "eagle_mpc/mpc-controllers/rail-mpc.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

void exposeRailMpc()
{
    bp::register_ptr_to_python<boost::shared_ptr<RailMpc>>();
    bp::class_<RailMpc, bp::bases<MpcAbstract>>(
        "RailMpc", bp::init<const std::vector<Eigen::VectorXd>&, const std::size_t, const std::string&>())
        .def("createProblem", &RailMpc::createProblem, bp::args("self"))
        .def("updateProblem", &RailMpc::updateProblem, bp::args("self", "currentTime"))
        .add_property("state_ref",
                      bp::make_function(&RailMpc::get_state_ref, bp::return_value_policy<bp::copy_const_reference>()),
                      "state trajectory")
        .add_property("t_ref",
                      bp::make_function(&RailMpc::get_t_ref, bp::return_value_policy<bp::copy_const_reference>()),
                      "time trajectory");
}

}  // namespace python
}  // namespace eagle_mpc
#endif