#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_ALGORITHMS_WAYPOINT_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_ALGORITHMS_WAYPOINT_HPP_

#include <boost/python.hpp>

#include "multicopter_mpc/ocp/ocp-base.hpp"

namespace multicopter_mpc {
namespace python {
namespace bp = boost::python;

class OcpAbstract_wrap : public OcpAbstract, public bp::wrapper<OcpAbstract> {
 public:
  using OcpAbstract::diff_model_terminal_;
  using OcpAbstract::diff_models_running_;
  using OcpAbstract::int_model_terminal_;
  using OcpAbstract::int_models_running_;
  using OcpAbstract::problem_;
  using OcpAbstract::solver_;

  OcpAbstract_wrap(const boost::shared_ptr<pinocchio::Model> model,
                   const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt)
      : OcpAbstract(model, mc_params, dt), bp::wrapper<OcpAbstract>() {}

  void createProblem(const SolverTypes::Type& solver_type) {
    return bp::call<void>(this->get_override("createProblem").ptr(), solver_type);
  }

  void loadParameters(const std::string& yaml_path) {
    return bp::call<void>(this->get_override("loadParameters").ptr(), yaml_path);
  }

  void solve() {
    if (bp::override solve = this->get_override("solver")) {
      return bp::call<void>(solve.ptr());
    }
    return OcpAbstract::solve();
  }
  void default_solve() { return this->OcpAbstract::solve(); }
};

void exposeOcpAbstract() {
  bp::enum_<SolverTypes::Type>("SolverType")
      .value("SolverTypeBoxFDDP", SolverTypes::BoxFDDP)
      .value("SolverTypeSquashBoxFDDP", SolverTypes::SquashBoxFDDP);

  bp::class_<OcpAbstract_wrap, boost::noncopyable>(
      "OcpAbstract",
      "Abstract class to generate an optimal control problem designed to run on multicopter or aerial manipulators",
      bp::init<const boost::shared_ptr<pinocchio::Model>&, const boost::shared_ptr<MultiCopterBaseParams>&,
               const double&>(bp::args("self", "model", "mc_params", "dt"),
                              "Initialize the Optimal Control problem abstract class"))
      .def("createProblem", pure_virtual(&OcpAbstract_wrap::createProblem), bp::args("self", "solver_type"))
      .def("loadParameters", pure_virtual(&OcpAbstract_wrap::loadParameters), bp::args("self", "yaml_path"))
      .def("setSolverCallbacks", &OcpAbstract_wrap::setSolverCallbacks, bp::args("self", "activated"))
      .def("solve", &OcpAbstract::solve, &OcpAbstract_wrap::default_solve, bp::args("self"))
      .def("setSolverIters", &OcpAbstract_wrap::setSolverIters, bp::args("self", "num_iters"))
      .add_property("model",
                    bp::make_function(&OcpAbstract_wrap::getModel, bp::return_value_policy<bp::return_by_value>()))
      .add_property("mc_params",
                    bp::make_function(&OcpAbstract_wrap::getMcParams, bp::return_value_policy<bp::return_by_value>()))
      .add_property("state",
                    bp::make_function(&OcpAbstract_wrap::getStateMultibody, bp::return_value_policy<bp::return_by_value>()))
      .add_property("actuation",
                    bp::make_function(&OcpAbstract_wrap::getActuation, bp::return_value_policy<bp::return_by_value>()))
      .add_property("problem",
                    bp::make_function(&OcpAbstract_wrap::getProblem, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&OcpAbstract_wrap::problem_))
      .add_property("solver",
                    bp::make_function(&OcpAbstract_wrap::getSolver, bp::return_value_policy<bp::return_by_value>()))
      .add_property("dt",
                    bp::make_function(&OcpAbstract_wrap::getTimeStep, bp::return_value_policy<bp::return_by_value>()))
      .add_property("u_lb", bp::make_function(&OcpAbstract_wrap::getActuationLowerBounds,
                                              bp::return_value_policy<bp::return_by_value>()))
      .add_property("u_ub", bp::make_function(&OcpAbstract_wrap::getActuationUpperBounds,
                                              bp::return_value_policy<bp::return_by_value>()))
      .add_property("base_link_id", bp::make_function(&OcpAbstract_wrap::getBaseLinkId,
                                                      bp::return_value_policy<bp::return_by_value>()))
      .add_property("n_knots",
                    bp::make_function(&OcpAbstract_wrap::getKnots, bp::return_value_policy<bp::return_by_value>()))
      .def("setInitialState", &OcpAbstract_wrap::setInitialState, bp::args("self", "initial_state"))
      .add_property(
          "diff_models_running",
          bp::make_getter(&OcpAbstract_wrap::diff_models_running_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&OcpAbstract_wrap::diff_models_running_))
      .add_property(
          "int_models_running",
          bp::make_getter(&OcpAbstract_wrap::int_models_running_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&OcpAbstract_wrap::int_models_running_))
      .add_property(
          "diff_model_terminal",
          bp::make_getter(&OcpAbstract_wrap::diff_model_terminal_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&OcpAbstract_wrap::diff_model_terminal_))
      .add_property(
          "int_model_terminal",
          bp::make_getter(&OcpAbstract_wrap::int_model_terminal_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&OcpAbstract_wrap::int_model_terminal_));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_ALGORITHMS_WAYPOINT_HPP_