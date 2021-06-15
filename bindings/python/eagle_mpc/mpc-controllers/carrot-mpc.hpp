///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_CARROT_MPC_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_CONTROLLERS_CARROT_MPC_HPP_

#include "eagle_mpc/mpc-controllers/carrot-mpc.hpp"

namespace eagle_mpc {
namespace python {

namespace bp = boost::python;

void exposeCarrotMpc() {
  bp::register_ptr_to_python<boost::shared_ptr<CarrotMpc>>();
  bp::class_<CarrotMpc, bp::bases<MpcAbstract>>(
      "CarrotMpc", bp::init<const boost::shared_ptr<Trajectory>&, const std::vector<Eigen::VectorXd>&,
                            const std::size_t, const std::string&>())
      .def("createProblem", &CarrotMpc::createProblem, bp::args("self"))
      .def("updateProblem", &CarrotMpc::updateProblem, bp::args("self", "currentTime"))
      .add_property("trajectory",
                    bp::make_function(&CarrotMpc::get_trajectory, bp::return_value_policy<bp::return_by_value>()))
      .add_property("state_ref",
                    bp::make_function(&CarrotMpc::get_state_ref, bp::return_value_policy<bp::copy_const_reference>()),
                    "state trajectory")
      .add_property("t_stages",
                    bp::make_function(&CarrotMpc::get_t_stages, bp::return_value_policy<bp::copy_const_reference>()),
                    "time trajectory")
      .add_property("t_ref",
                    bp::make_function(&CarrotMpc::get_t_ref, bp::return_value_policy<bp::copy_const_reference>()),
                    "time trajectory");
}

}  // namespace python
}  // namespace eagle_mpc
#endif