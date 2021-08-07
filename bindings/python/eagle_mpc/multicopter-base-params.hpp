///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_EAGLE_MPC_MULTICOPTER_BASE_PARAMS_HPP_
#define BINDINGS_PYTHON_EAGLE_MPC_MULTICOPTER_BASE_PARAMS_HPP_

#include <boost/python.hpp>
#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

#include "python/eagle_mpc/utils/vector-converter.hpp"
#include "eagle_mpc/multicopter-base-params.hpp"

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

void exposeMultiCopterBaseParams()
{
    bp::register_ptr_to_python<boost::shared_ptr<MultiCopterBaseParams>>();

    StdVectorPythonVisitor<pinocchio::SE3, std::allocator<pinocchio::SE3>, true>::expose("StdVec_Rotors");

    void (MultiCopterBaseParams::*auto_setup)(const std::string&, const boost::shared_ptr<ParamsServer>&,
                                              const boost::shared_ptr<pinocchio::Model>&) =
        &MultiCopterBaseParams::autoSetup;

    void (MultiCopterBaseParams::*auto_setup_nomodel)(const std::string&, const boost::shared_ptr<ParamsServer>&) =
        &MultiCopterBaseParams::autoSetup;

    bp::class_<MultiCopterBaseParams>(
        "MultiCopterBaseParams",
        bp::init<const double&, const double&, const Eigen::MatrixXd&, const double&, const double&,
                 const std::string&>(
            bp::args("self", "cf", "cm", "torque_force", "max_th", "min_th", "base_link_name"),
            "Initialize multicopter params"))
        .def(bp::init<>(bp::args("self"), "Default initialization"))
        .def("autoSetup", auto_setup, bp::args("self", "path_to_platform", "params_server", "robot_model"))
        .def("autoSetup", auto_setup_nomodel, bp::args("self", "path_to_platform", "params_server"))
        .add_property("cf",
                      bp::make_getter(&MultiCopterBaseParams::cf_, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&MultiCopterBaseParams::cf_), "cf coefficient")
        .add_property("cm",
                      bp::make_getter(&MultiCopterBaseParams::cm_, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&MultiCopterBaseParams::cm_), "cm coefficient")
        .add_property(
            "n_rotors",
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
            "base_link_name",
            bp::make_getter(&MultiCopterBaseParams::base_link_name_, bp::return_value_policy<bp::return_by_value>()),
            bp::make_setter(&MultiCopterBaseParams::base_link_name_), "base link name")
        .add_property(
            "max_torque",
            bp::make_getter(&MultiCopterBaseParams::max_torque_, bp::return_value_policy<bp::return_by_value>()),
            bp::make_setter(&MultiCopterBaseParams::max_torque_), "max torque for arm joints")
        .add_property(
            "min_torque",
            bp::make_getter(&MultiCopterBaseParams::min_torque_, bp::return_value_policy<bp::return_by_value>()),
            bp::make_setter(&MultiCopterBaseParams::min_torque_), "min torque for arm joints")
        .add_property("u_lb",
                      bp::make_getter(&MultiCopterBaseParams::u_lb, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&MultiCopterBaseParams::u_lb), "u_lb")
        .add_property("u_ub",
                      bp::make_getter(&MultiCopterBaseParams::u_ub, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&MultiCopterBaseParams::u_ub), "u_ub")
        .add_property("rotors_pose", bp::make_getter(&MultiCopterBaseParams::rotors_pose_,
                                                     bp::return_value_policy<bp::return_by_value>()));
}
}  // namespace python
}  // namespace eagle_mpc

#endif  // BINDINGS_PYTHON_EAGLE_MPC_MULTICOPTER_BASE_PARAMS_HPP_
