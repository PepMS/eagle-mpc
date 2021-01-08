#ifndef PYTHON_BINDINGS
#define PYTHON_BINDINGS

#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

#include "python/multicopter_mpc/waypoint.hpp"
#include "python/multicopter_mpc/multicopter-base-params.hpp"
#include "python/multicopter_mpc/mission.hpp"
#include "python/multicopter_mpc/ocp/trajectory-generator.hpp"
#include "python/multicopter_mpc/ocp-base.hpp"
#include "python/multicopter_mpc/main-mpc.hpp"
#include "python/multicopter_mpc/ocp/mpc-base.hpp"
#include "python/multicopter_mpc/sbfddp.hpp"

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libmulticopter_mpc_pywrap) {
  // eigenpy::enableEigenPy();

  exposeWayPoint();
  exposeMultiCopterBaseParams();
  exposeMission();
  exposeOcpAbstract();
  exposeTrajectoryGenerator();
  exposeMpcMain();
  exposeMpcAbstract();
  exposeSolverSbFDDP();
}

}  // namespace python
}  // namespace multicopter_mpc

#endif