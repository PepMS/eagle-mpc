
#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_

#include "multicopter_mpc/utils/parser_yaml.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeParserYaml() {
  bp::class_<ParserYaml>("ParserYaml", bp::init<std::string, std::string>(
    bp::args("file", "path_root", "freely_parse"), "Initialize parser YAML"))
    .def("get_params", &ParserYaml::get_params,bp::return_value_policy<bp::reference_existing_object>());
}

}  // namespace python
}  // namespace optiuavm

#endif  // BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_