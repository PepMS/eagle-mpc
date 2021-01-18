#include "multicopter_mpc/factory/cost.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

const std::map<std::string, CostModelTypes::Type> CostModelTypes::all = CostModelTypes::init_all();

CostModelFactory::CostModelFactory() {}
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(const std::string& path_to_cost,
                                                                         const ParamsServer& server,
                                                                         const boost::shared_ptr<Stage>& stage) const {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::ActivationModelAbstract> activation;
  Eigen::VectorXd reference;
  switch (CostModelTypes::all.at(server.getParam<std::string>(path_to_cost + "type"))) {
    case CostModelTypes::CostModelState:
      activation =
          activation_factory_->create(path_to_cost, server, stage->get_trajectory()->get_robot_state()->get_ndx());
      try {
        reference = converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "reference"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero state vector";
        reference = stage->get_trajectory()->get_robot_state()->zero();
      }
      if (reference.size() != stage->get_trajectory()->get_robot_state()->get_nx()) {
        throw std::runtime_error("State reference vector @" + path_to_cost + "reference has dimension " +
                                 std::to_string(reference.size()) + ". Should be " +
                                 std::to_string(stage->get_trajectory()->get_robot_state()->get_nx()));
      }
      cost =
          boost::make_shared<crocoddyl::CostModelState>(stage->get_trajectory()->get_robot_state(), activation,
                                                        reference, stage->get_trajectory()->get_actuation()->get_nu());
      break;
    default:
      break;
  }
  return cost;
}

}  // namespace multicopter_mpc