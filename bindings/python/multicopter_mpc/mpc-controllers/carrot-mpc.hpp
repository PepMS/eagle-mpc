#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_CONTROLLERS_CARROT_MPC_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_CONTROLLERS_CARROT_MPC_HPP_

#include "multicopter_mpc/mpc-controllers/carrot-mpc.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeCarrotMpc() {
  bp::register_ptr_to_python<boost::shared_ptr<CarrotMpc>>();
  bp::class_<CarrotMpc, bp::bases<MpcAbstract>>(
      "CarrotMpc", bp::init<const boost::shared_ptr<Trajectory>&, const std::string&>())
      .def("createProblem", &CarrotMpc::createProblem, bp::args("self"));
}

}  // namespace python
}  // namespace multicopter_mpc
#endif