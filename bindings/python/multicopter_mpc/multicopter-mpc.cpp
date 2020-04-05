#ifndef PYTHON_BINDINGS
#define PYTHON_BINDINGS

#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libmulticopter_mpc_pywrap) {
  bp::class_<std::map<std::string, std::string> >("StringMap")
      .def(bp::map_indexing_suite<std::map<std::string, std::string> >());

}

}  // namespace python
}  // namespace optiuavm

#endif