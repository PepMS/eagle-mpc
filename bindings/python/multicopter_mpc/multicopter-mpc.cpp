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

#include "python/eagle_mpc/utils/vector-converter.hpp"
#include "python/eagle_mpc/utils/map-converter.hpp"

#include "python/eagle_mpc/multicopter-base-params.hpp"
#include "python/eagle_mpc/mpc-base.hpp"
#include "python/eagle_mpc/mpc-controllers/carrot-mpc.hpp"
#include "python/eagle_mpc/mpc-controllers/rail-mpc.hpp"
#include "python/eagle_mpc/mpc-controllers/weighted-mpc.hpp"
#include "python/eagle_mpc/sbfddp.hpp"
#include "python/eagle_mpc/stage.hpp"
#include "python/eagle_mpc/trajectory.hpp"
#include "python/eagle_mpc/utils/parser_yaml.hpp"
#include "python/eagle_mpc/utils/params_server.hpp"

namespace eagle_mpc {
namespace python {

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libeagle_mpc_pywrap) {
  exposeMpcAbstract();
  exposeCarrotMpc();
  exposeRailMpc();
  exposeWeightedMpc();
  exposeMultiCopterBaseParams();
  exposeSolverSbFDDP();
  exposeTrajectory();
  exposeParamsServer();
  exposeParserYaml();
  exposeStage();
}

}  // namespace python
}  // namespace eagle_mpc

#endif