#ifndef BINDINGS_PYTHON_MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_

#include "multicopter_mpc/multicopter-base-params.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeMultiCopterBaseParams() {
  bp::register_ptr_to_python<boost::shared_ptr<MultiCopterBaseParams> >();

  bp::class_<MultiCopterBaseParams>(
      "MultiCopterBaseParams",
      bp::init<double, double, Eigen::MatrixXd, double, double>(
          bp::args("cf", "cm", "torque_force", "max_th", "min_th"), "Initialize multicopter params"))
      .def(bp::init<double, double, Eigen::MatrixXd, double, double, Eigen::VectorXd, Eigen::VectorXd>(
          bp::args("cf", "cm", "torque_force", "max_th", "min_th", "max torque", "min torque"),
          "Initialize multicopter params"))
      .def(bp::init<>())
      .def("fill", &MultiCopterBaseParams::fill, bp::args("server"))
      .add_property("cf", bp::make_getter(&MultiCopterBaseParams::cf_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&MultiCopterBaseParams::cf_), "cf coefficient")
      .add_property("cm", bp::make_getter(&MultiCopterBaseParams::cm_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&MultiCopterBaseParams::cm_), "cm coefficient")
      .add_property("n_rotors",
                    bp::make_getter(&MultiCopterBaseParams::n_rotors_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&MultiCopterBaseParams::n_rotors_), "n rotors")
      .add_property("tau_f",
                    bp::make_getter(&MultiCopterBaseParams::tau_f_, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_setter(&MultiCopterBaseParams::tau_f_), "tau_f matrix")
      .add_property(
          "max_thrust",
          bp::make_getter(&MultiCopterBaseParams::max_thrust_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&MultiCopterBaseParams::max_thrust_), "max thrust")
      .add_property(
          "min_thrust",
          bp::make_getter(&MultiCopterBaseParams::min_thrust_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&MultiCopterBaseParams::min_thrust_), "min thrust")
      .add_property(
          "max_torque",
          bp::make_getter(&MultiCopterBaseParams::max_torque_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&MultiCopterBaseParams::max_torque_), "max torque")
      .add_property(
          "min_torque",
          bp::make_getter(&MultiCopterBaseParams::min_torque_, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&MultiCopterBaseParams::min_torque_), "min torque");
  

}
}  // namespace python
}  // namespace multicopter_mpc

#endif  // BINDINGS_PYTHON_MULTICOPTER_MPC_MULTICOPTER_BASE_PARAMS_HPP_
