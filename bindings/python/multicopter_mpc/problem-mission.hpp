#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_PROBLEM_MISSION_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_PROBLEM_MISSION_HPP_

#include "multicopter_mpc/problem-mission.hpp"

#include <Eigen/Dense>
// #include <eigenpy/eigenpy.hpp>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeProblemMission() {
  boost::shared_ptr<crocoddyl::ShootingProblem> (ProblemMission::*createProblem3)() = &ProblemMission::createProblem;

  bp::class_<ProblemMission>("ProblemMission",
                             bp::init<boost::shared_ptr<Mission>, boost::shared_ptr<MultiCopterBaseParams>,
                                      boost::shared_ptr<pinocchio::Model>, int, double>(
                                 bp::args("mission", "mc_params", "mc_model", "frame_id", "dt"), "Initialize mission problem"))
      .def("createProblem", createProblem3);
}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_
