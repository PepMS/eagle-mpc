#ifndef BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_
#define BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_

#include "yaml_parser/parser_yaml.hpp"

namespace yaml_parser {
namespace python {

namespace bp = boost::python;

void exposeParserYaml() {
  bp::class_<ParserYAML>("ParserYAML", bp::init<std::string, std::string, bool>(
    bp::args("file", "path_root", "freely_parse"), "Initialize parser YAML"))
    .def("getParams", &ParserYAML::getParams,bp::return_value_policy<bp::return_by_value>());
}

}  // namespace python
}  // namespace optiuavm

#endif  // BINDINGS_PYTHON_OPTIUAVM_YAML_PARSER_YAML_HPP_