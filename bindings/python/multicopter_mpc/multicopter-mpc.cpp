#ifndef PYTHON_BINDINGS
#define PYTHON_BINDINGS

#include <Eigen/Dense>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/enum.hpp>

#include <eigenpy/eigenpy.hpp>
#include "pinocchio/fwd.hpp"
#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/multibody/fwd.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"
#include "python/multicopter_mpc/utils/map-converter.hpp"

#include "python/multicopter_mpc/multicopter-base-params.hpp"
#include "python/multicopter_mpc/mpc-base.hpp"
#include "python/multicopter_mpc/mpc-controllers/carrot-mpc.hpp"
#include "python/multicopter_mpc/sbfddp.hpp"
#include "python/multicopter_mpc/stage.hpp"
#include "python/multicopter_mpc/trajectory.hpp"
#include "python/multicopter_mpc/utils/parser_yaml.hpp"
#include "python/multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libmulticopter_mpc_pywrap) {
  exposeMpcAbstract();
  exposeCarrotMpc();
  exposeMultiCopterBaseParams();
  exposeSolverSbFDDP();
  exposeTrajectory();
  exposeParamsServer();
  exposeParserYaml();
  exposeStage();
}

}  // namespace python
}  // namespace multicopter_mpc

#endif