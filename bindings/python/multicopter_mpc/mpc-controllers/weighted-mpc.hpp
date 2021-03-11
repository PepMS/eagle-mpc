#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_

#include "multicopter_mpc/mpc-controllers/weighted-mpc.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeWeightedMpc() {
  bp::register_ptr_to_python<boost::shared_ptr<WeightedMpc>>();
  bp::class_<WeightedMpc, bp::bases<MpcAbstract>>(
      "WeightedMpc", bp::init<const boost::shared_ptr<Trajectory>&, const std::size_t, const std::string&>())
      .def("createProblem", &WeightedMpc::createProblem, bp::args("self"))
      .def("updateProblem", &WeightedMpc::updateProblem, bp::args("self", "currentTime"))
      .add_property("trajectory",
                    bp::make_function(&WeightedMpc::get_trajectory, bp::return_value_policy<bp::return_by_value>()))
      .add_property("t_stages",
                    bp::make_function(&WeightedMpc::get_t_stages, bp::return_value_policy<bp::copy_const_reference>()),
                    "time trajectory");
}

}  // namespace python
}  // namespace multicopter_mpc
#endif