#ifndef BINDINGS_PYTHON_MULTICOPTER_STAGE_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_STAGE_HPP_

#include "eagle_mpc/stage.hpp"

#include "python/eagle_mpc/utils/vector-converter.hpp"
#include "python/eagle_mpc/utils/map-converter.hpp"

#include <Eigen/Dense>

namespace eagle_mpc {
namespace python {

namespace bp = boost::python;

void exposeStage() {
  bp::enum_<CostModelTypes>("CostModelTypes")
      .value("CostModelState", CostModelTypes::CostModelState)
      .value("CostModelControl", CostModelTypes::CostModelControl)
      .value("CostModelFramePlacement", CostModelTypes::CostModelFramePlacement)
      .value("CostModelFrameTranslation", CostModelTypes::CostModelFrameTranslation)
      .value("CostModelFrameVelocity", CostModelTypes::CostModelFrameVelocity)
      .value("CostModelContactFrictionCone", CostModelTypes::CostModelContactFrictionCone);

  bp::enum_<ContactModelTypes>("ContactModelTypes")
      .value("ContactModel2D", ContactModelTypes::ContactModel2D)
      .value("ContactModel3D", ContactModelTypes::ContactModel3D)
      .value("ContactModel6D", ContactModelTypes::ContactModel6D);

  StdMapPythonVisitor<std::string, CostModelTypes, std::less<std::string>,
                      std::allocator<std::pair<const std::string, CostModelTypes>>, true>::expose("StdMap_CostType");
  StdMapPythonVisitor<std::string, ContactModelTypes, std::less<std::string>,
                      std::allocator<std::pair<const std::string, ContactModelTypes>>,
                      true>::expose("StdMap_ContactType");

  bp::class_<Stage, boost::shared_ptr<Stage>>("Stage", bp::no_init)
      .def("__init__", bp::make_constructor(&Stage::create))
      .add_property("trajectory",
                    bp::make_function(&Stage::get_trajectory, bp::return_value_policy<bp::return_by_value>()),
                    "parent trajectory")
      .add_property("costs", bp::make_function(&Stage::get_costs, bp::return_value_policy<bp::return_by_value>()),
                    "cost model sum for the given stage")
      .add_property("contacts",
                    bp::make_function(&Stage::get_contacts, bp::return_value_policy<bp::return_by_value>()),
                    "contact model multiple for the given stage")
      .add_property("duration",
                    bp::make_function(&Stage::get_duration, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_function(&Stage::set_duration), "duration of the stage in ms")
      .add_property("t_ini", bp::make_function(&Stage::get_t_ini, bp::return_value_policy<bp::return_by_value>()),
                    bp::make_function(&Stage::set_t_ini), "initial time of the stage in ms")
      .add_property("is_terminal",
                    bp::make_function(&Stage::get_is_terminal, bp::return_value_policy<bp::return_by_value>()),
                    "true if the stage is terminal")
      .add_property("is_transition",
                    bp::make_function(&Stage::get_is_transition, bp::return_value_policy<bp::return_by_value>()),
                    "true if the stage is terminal")
      .add_property("name", bp::make_function(&Stage::get_name, bp::return_value_policy<bp::return_by_value>()),
                    "name of the stage")
      .add_property("cost_types",
                    bp::make_function(&Stage::get_cost_types, bp::return_value_policy<bp::return_by_value>()),
                    "existing cost types")
      .add_property("contact_types",
                    bp::make_function(&Stage::get_contact_types, bp::return_value_policy<bp::return_by_value>()),
                    "existing contact types")
      .def("autoSetup", &Stage::autoSetup, bp::args("self", "stages", "server"));
}
}  // namespace python
}  // namespace eagle_mpc
#endif