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
    case CostModelTypes::CostModelState: {
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
    } break;
    case CostModelTypes::CostModelControl: {
      activation =
          activation_factory_->create(path_to_cost, server, stage->get_trajectory()->get_actuation()->get_nu());

      try {
        reference = converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "reference"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero control vector";
        reference = Eigen::VectorXd::Zero(stage->get_trajectory()->get_actuation()->get_nu());
      }
      if (reference.size() != stage->get_trajectory()->get_actuation()->get_nu()) {
        throw std::runtime_error("Control reference vector @" + path_to_cost + "reference has dimension " +
                                 std::to_string(reference.size()) + ". Should be " +
                                 std::to_string(stage->get_trajectory()->get_actuation()->get_nu()));
      }

      cost = boost::make_shared<crocoddyl::CostModelControl>(stage->get_trajectory()->get_robot_state(), activation,
                                                             reference);
    } break;
    case CostModelTypes::CostModelFramePlacement: {
      activation = activation_factory_->create(path_to_cost, server, 6);

      Eigen::Vector3d position =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "position"));
      Eigen::Vector4d orientation =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "orientation"));
      std::string link_name = server.getParam<std::string>(path_to_cost + "link_name");

      Eigen::Quaterniond quat(orientation);
      quat.normalize();
      pinocchio::SE3 m_ref(quat.toRotationMatrix(), position);

      crocoddyl::FramePlacement frame(stage->get_trajectory()->get_robot_model()->getFrameId(link_name), m_ref);
      cost = boost::make_shared<crocoddyl::CostModelFramePlacement>(stage->get_trajectory()->get_robot_state(),
                                                                    activation, frame);
    } break;
    case CostModelTypes::CostModelFrameVelocity: {
      activation = activation_factory_->create(path_to_cost, server, 6);

      Eigen::Vector3d linear =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "linear"));
      Eigen::Vector3d angular =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_cost + "angular"));
      std::string link_name = server.getParam<std::string>(path_to_cost + "link_name");

      pinocchio::Motion motion_ref(linear, angular);

      crocoddyl::FrameMotion frame(stage->get_trajectory()->get_robot_model()->getFrameId(link_name), motion_ref);
      cost = boost::make_shared<crocoddyl::CostModelFrameVelocity>(stage->get_trajectory()->get_robot_state(),
                                                                   activation, frame);
    } break;
    default:
      throw std::runtime_error("Cost " + server.getParam<std::string>(path_to_cost + "type") +
                               "not found. Please make sure the specified cost exists.");
      break;
  }
  return cost;
}

}  // namespace multicopter_mpc