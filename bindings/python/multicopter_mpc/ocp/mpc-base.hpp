#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_OCP_MPC_BASE_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_OCP_MPC_BASE_HPP_

#include <boost/python.hpp>

#include "multicopter_mpc/ocp/mpc/mpc-base.hpp"

namespace multicopter_mpc {
namespace python {
namespace bp = boost::python;

class MpcAbstract_wrap : public MpcAbstract, public bp::wrapper<MpcAbstract> {
 public:
  MpcAbstract_wrap(const boost::shared_ptr<pinocchio::Model> model,
                   const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                   const boost::shared_ptr<Mission>& mission)
      : MpcAbstract(model, mc_params, mission), bp::wrapper<MpcAbstract>() {}

  void createProblem(const SolverTypes::Type& solver_type, const IntegratorTypes::Type& integrator_type) {
    return bp::call<void>(this->get_override("createProblem").ptr(), solver_type, integrator_type);
  }

  void setTimeStep(const double& dt) { return bp::call<void>(this->get_override("setTimeStep").ptr(), dt); }

  void updateProblem(const std::size_t& idx_trajectory) {
    return bp::call<void>(this->get_override("updateProblem").ptr(), idx_trajectory);
  }

  void loadParameters(const std::string& yaml_path) {
    if (bp::override load_parameters = this->get_override("loadParameters")) {
      return bp::call<void>(load_parameters.ptr(), yaml_path);
    }
    return this->MpcAbstract::loadParameters(yaml_path);
  }
  void default_loadParameters(const std::string& yaml_path) { return this->MpcAbstract::loadParameters(yaml_path); }

  void setNumberKnots(const std::size_t& n_knots) {
    if (bp::override set_number_knots = this->get_override("setNumberKnots")) {
      return bp::call<void>(set_number_knots.ptr(), n_knots);
    }
    return this->MpcAbstract::setNumberKnots(n_knots);
  }
  void default_setNumberKnots(const std::size_t& n_knots) { return this->MpcAbstract::setNumberKnots(n_knots); }

  void printInfo() {
    if (bp::override print_info = this->get_override("printInfo")) {
      return bp::call<void>(print_info.ptr());
    }
    return this->MpcAbstract::printInfo();
  }
  void default_printInfo() { return this->MpcAbstract::printInfo(); }
};

void exposeMpcAbstract() {
  bp::register_ptr_to_python<boost::shared_ptr<MpcAbstract>>();
  bp::register_ptr_to_python<boost::shared_ptr<MpcAbstract const>>();
  bp::class_<MpcAbstract_wrap, boost::noncopyable, bp::bases<OcpAbstract>>(
      "MpcAbstract",
      "Abstract class to generate an MPC controller with the typical methods existing in an MPC algorithm",
      bp::init<const boost::shared_ptr<pinocchio::Model>&, const boost::shared_ptr<MultiCopterBaseParams>&,
               const boost::shared_ptr<Mission>&>(bp::args("self", "model", "mc_params", "mission"),
                                                  "Initialize the MPC Controller class"))
      .def("loadParameters", &MpcAbstract_wrap::loadParameters, &MpcAbstract_wrap::default_loadParameters,
           bp::args("self", "yaml_path"))
      .def("setNumberKnots", &MpcAbstract_wrap::setNumberKnots, &MpcAbstract_wrap::default_setNumberKnots,
           bp::args("self", "n_knots"))
      .def("printInfo", &MpcAbstract_wrap::printInfo, &MpcAbstract_wrap::default_printInfo, bp::args("self"))
      .add_property("mission",
                    bp::make_function(&MpcAbstract_wrap::getMission, bp::return_value_policy<bp::return_by_value>()))
      .add_property("trajectory_generator", bp::make_function(&MpcAbstract_wrap::getTrajectoryGenerator,
                                                              bp::return_value_policy<bp::return_by_value>()));
}

}  // namespace python
}  // namespace multicopter_mpc

#endif