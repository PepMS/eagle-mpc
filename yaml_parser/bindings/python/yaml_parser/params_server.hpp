#ifndef BINDINGS_PYTHON_YAML_PARSER_YAML_PARAMS_SERVER_HPP_
#define BINDINGS_PYTHON_YAML_PARSER_YAML_PARAMS_SERVER_HPP_

#include "yaml_parser/params_server.hpp"

namespace yaml_parser {
namespace python {

namespace bp = boost::python;

void exposeParamsServer() {
  bp::class_<ParamsServer>("ParamsServer", bp::init<std::map<std::string, std::string>>(
    bp::args("params"), "Initialize params server"));
}

}  // namespace python
}  // namespace optiuavm

#endif  // BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_