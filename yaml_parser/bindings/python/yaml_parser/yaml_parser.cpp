#ifndef PYTHON_BINDINGS
#define PYTHON_BINDINGS

// #include "pinocchio/fwd.hpp"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/enum.hpp>

#include "python/yaml_parser/params_server.hpp"
#include "python/yaml_parser/parser_yaml.hpp"

namespace yaml_parser {
namespace python {

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libyaml_parser_pywrap) {
  bp::class_<std::map<std::string, std::string> >("StringMap")
      .def(bp::map_indexing_suite<std::map<std::string, std::string> >());
  exposeParamsServer();
  exposeParserYaml();
}

}  // namespace python
}  // namespace yaml_parser

#endif