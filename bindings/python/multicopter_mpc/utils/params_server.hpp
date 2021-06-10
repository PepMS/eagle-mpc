
#ifndef BINDINGS_PYTHON_EAGLE_MPC_UTILS_PARAMS_SERVER_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_UTILS_PARAMS_SERVER_HPP_

#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeParamsServer() {
  bp::class_<std::map<std::string, std::string>>("StringMap")
      .def(bp::map_indexing_suite<std::map<std::string, std::string>>());
  bp::class_<ParamsServer>(
      "ParamsServer", bp::init<std::map<std::string, std::string>>(bp::args("params"), "Initialize params server"));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_