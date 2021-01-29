#ifndef BINDINGS_PYTHON_MULTICOPTER_STAGE_HPP_
#define BINDINGS_PYTHON_MULTICOPTER_STAGE_HPP_

#include "multicopter_mpc/stage.hpp"

#include "python/multicopter_mpc/utils/vector-converter.hpp"

#include <Eigen/Dense>

namespace multicopter_mpc {
namespace python {

namespace bp = boost::python;

void exposeStage() {
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
                    "duration of the stage in ms")
      .add_property("is_terminal",
                    bp::make_function(&Stage::get_is_terminal, bp::return_value_policy<bp::return_by_value>()),
                    "true if the stage is terminal")
      .add_property("name", bp::make_function(&Stage::get_name, bp::return_value_policy<bp::return_by_value>()),
                    "name of the stage")

      .def("autoSetup", &Stage::autoSetup, bp::args("self", "stages", "server"));
}
}  // namespace python
}  // namespace multicopter_mpc
#endif